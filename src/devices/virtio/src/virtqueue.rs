// Copyright (c) 2021 Intel Corporation
//
// SPDX-License-Identifier: BSD-2-Clause-Patent

use alloc::vec::Vec;
use bitflags::bitflags;
use core::mem::size_of;
use core::slice;
use core::sync::atomic::{fence, Ordering};
use volatile::Volatile;

use crate::{Result, VirtioError, VirtioTransport, PAGE_SIZE};

const MAX_QUEUE_SIZE: usize = 32;

#[derive(Debug)]
pub struct VirtqueueBuf {
    pub addr: u64,
    pub len: u32,
}

impl VirtqueueBuf {
    pub fn new(addr: u64, len: u32) -> Self {
        Self { addr, len }
    }
}

/// The mechanism for bulk data transport on virtio devices.
///
/// Each device can have zero or more virtqueues.
#[repr(C)]
pub struct VirtQueue {
    /// Descriptor table
    desc: &'static mut [Descriptor],
    /// Available ring
    avail: &'static mut AvailRing,
    /// Used ring
    used: &'static mut UsedRing,

    /// The index of queue
    queue_idx: u32,
    /// The size of queue
    queue_size: u16,
    /// The number of used queues.
    num_used: u16,
    /// The head desc index of the free list.
    free_head: u16,
    avail_idx: u16,
    last_used_idx: u16,

    /// Private shadow of the descriptor `next` links. This is the authoritative
    /// source for free-list traversal and chain recycling; the host-writable
    /// `desc[].next` field in shared DMA is never trusted to drive guest-side
    /// accounting.
    desc_next: [u16; MAX_QUEUE_SIZE],
    /// Private shadow of each descriptor's direction (true = device-writable,
    /// i.e. host-to-guest). Recorded at `add` time so recycling does not rely on
    /// the host-writable `desc[].flags`.
    desc_write: [bool; MAX_QUEUE_SIZE],
    /// Number of descriptors in the in-flight chain headed by each index. A
    /// value of 0 means the index is not the head of an in-flight chain, so a
    /// host-reported used id that is not a real chain head is rejected.
    chain_len: [u16; MAX_QUEUE_SIZE],
}

impl VirtQueue {
    /// Create a new VirtQueue.
    pub fn new(
        header: &dyn VirtioTransport,
        idx: usize,
        dma_addr: u64,
        queue_size: u16,
    ) -> Result<Self> {
        // TBD: add get_descriptors_address for VirtioTransport
        // if header.queue_used(idx as u32) {
        //     return Err(Error::AlreadyUsed);
        // }
        if !queue_size.is_power_of_two()
            || header.get_queue_max_size()? < queue_size
            || queue_size > MAX_QUEUE_SIZE as u16
        {
            return Err(VirtioError::InvalidParameter);
        }
        let layout = VirtQueueLayout::new(queue_size).ok_or(VirtioError::CreateVirtioQueue)?;

        // header.queue_set(idx as u32, size as u32, PAGE_SIZE as u32, dma.pfn());
        header.set_descriptors_address(dma_addr)?;
        header.set_avail_ring(dma_addr + layout.avail_offset as u64)?;
        header.set_used_ring(dma_addr + layout.used_offset as u64)?;

        let desc: &'static mut [Descriptor] =
            unsafe { slice::from_raw_parts_mut(dma_addr as *mut Descriptor, queue_size as usize) };
        let avail: &'static mut AvailRing =
            unsafe { &mut *((dma_addr as usize + layout.avail_offset) as *mut AvailRing) };
        let used: &'static mut UsedRing =
            unsafe { &mut *((dma_addr as usize + layout.used_offset) as *mut UsedRing) };

        // link descriptors together
        let mut desc_next = [0u16; MAX_QUEUE_SIZE];
        for i in 0..(queue_size - 1) {
            desc[i as usize].next.write(i + 1);
            desc_next[i as usize] = i + 1;
        }

        Ok(VirtQueue {
            desc,
            avail,
            used,
            queue_size,
            queue_idx: idx as u32,
            num_used: 0,
            free_head: 0,
            avail_idx: 0,
            last_used_idx: 0,
            desc_next,
            desc_write: [false; MAX_QUEUE_SIZE],
            chain_len: [0; MAX_QUEUE_SIZE],
        })
    }

    /// Add DMA buffers to the virtqueue, return a token.
    pub fn add(&mut self, g2h: &[VirtqueueBuf], h2g: &[VirtqueueBuf]) -> Result<u16> {
        if g2h.is_empty() && h2g.is_empty() {
            return Err(VirtioError::InvalidParameter);
        }
        if g2h.len() + h2g.len() + self.num_used as usize > self.queue_size as usize {
            return Err(VirtioError::BufferTooSmall);
        }

        // allocate descriptors from free list
        let head = self.free_head;
        let mut last = self.free_head;

        for buf in g2h.iter() {
            last = self.add_descriptor(buf, DescFlags::NEXT)?;
        }

        for buf in h2g.iter() {
            last = self.add_descriptor(buf, DescFlags::NEXT | DescFlags::WRITE)?;
        }

        // Clear the 'NEXT' flag of the last added descriptor
        let desc = self
            .desc
            .get_mut(last as usize)
            .ok_or(VirtioError::InvalidDescriptorIndex)?;
        let mut flags = desc.flags.read();
        flags.remove(DescFlags::NEXT);
        desc.flags.write(flags);

        let chain_len = (g2h.len() + h2g.len()) as u16;
        self.num_used += chain_len;
        // Record the submitted chain length privately so recycling validates the
        // host-reported used id against a chain we actually built.
        self.chain_len[head as usize] = chain_len;

        let avail_slot = self.avail_idx & (self.queue_size - 1);
        self.avail
            .ring
            .get_mut(avail_slot as usize)
            .ok_or(VirtioError::InvalidDescriptorIndex)?
            .write(head);

        // write barrier
        fence(Ordering::SeqCst);

        // increase head of avail ring
        self.avail_idx = self.avail_idx.wrapping_add(1);
        self.avail.idx.write(self.avail_idx);
        self.avail.used_event.write(self.avail_idx);

        Ok(head)
    }

    /// Whether there is a used element that can pop.
    pub fn can_pop(&self) -> bool {
        self.last_used_idx != self.used.idx.read()
    }

    /// The number of free descriptors.
    pub fn available_desc(&self) -> usize {
        (self.queue_size - self.num_used) as usize
    }

    fn add_descriptor(&mut self, buf: &VirtqueueBuf, flag: DescFlags) -> Result<u16> {
        let index = self.free_head;
        let desc = self
            .desc
            .get_mut(index as usize)
            .ok_or(VirtioError::InvalidDescriptorIndex)?;
        desc.set_buf(buf);
        desc.flags.write(flag);

        // Advance the free head using the private shadow link instead of the
        // host-writable `desc.next`, and record the buffer direction privately.
        self.free_head = self.desc_next[index as usize];
        self.desc_write[index as usize] = flag.contains(DescFlags::WRITE);

        Ok(index)
    }

    /// Recycle descriptors in the list specified by head.
    ///
    /// This will push all linked descriptors at the front of the free list.
    fn recycle_descriptors(
        &mut self,
        head: u16,
        g2h: &mut Vec<VirtqueueBuf>,
        h2g: &mut Vec<VirtqueueBuf>,
    ) -> Result<()> {
        // Validate, against private state, that `head` (reported by the host via
        // the used ring) is the head of a chain we actually submitted. The walk
        // below is driven entirely by the private shadow, never by the
        // host-writable `desc.next`/`desc.flags`, so the host cannot redirect it
        // to never-submitted slots or leak in-flight descriptors.
        let len = self
            .chain_len
            .get(head as usize)
            .copied()
            .ok_or(VirtioError::InvalidDescriptorIndex)?;
        if len == 0 || len > self.num_used {
            return Err(VirtioError::InvalidDescriptor);
        }

        let origin_free_head = self.free_head;
        let mut cur = head;
        for i in 0..len {
            let index = cur as usize;
            let next = self.desc_next[index];
            let is_write = self.desc_write[index];
            let is_last = i + 1 == len;

            let desc = self
                .desc
                .get_mut(index)
                .ok_or(VirtioError::InvalidDescriptorIndex)?;
            let addr = desc.addr.read();
            let buf_len = desc.len.read();
            if is_last {
                // Relink the tail of the recycled chain to the previous free
                // head in the shared table so the device still observes a
                // consistent free list.
                desc.next.write(origin_free_head);
            }

            if is_write {
                h2g.push(VirtqueueBuf::new(addr, buf_len));
            } else {
                g2h.push(VirtqueueBuf::new(addr, buf_len));
            }

            if is_last {
                self.desc_next[index] = origin_free_head;
            }
            cur = next;
        }

        self.free_head = head;
        self.num_used -= len;
        self.chain_len[head as usize] = 0;

        Ok(())
    }

    /// Get a token from device used buffers, return (token, len).
    ///
    /// Ref: linux virtio_ring.c virtqueue_get_buf_ctx
    pub fn pop_used(
        &mut self,
        g2h: &mut Vec<VirtqueueBuf>,
        h2g: &mut Vec<VirtqueueBuf>,
    ) -> Result<u32> {
        if !self.can_pop() {
            return Err(VirtioError::NotReady);
        }
        // read barrier
        fence(Ordering::SeqCst);

        let last_used_idx = self.last_used_idx & (self.queue_size - 1);
        let last_used_slot = self
            .used
            .ring
            .get(last_used_idx as usize)
            .ok_or(VirtioError::InvalidRingIndex)?;
        let index = last_used_slot.id.read() as u16;
        let len = last_used_slot.len.read();

        self.recycle_descriptors(index, g2h, h2g)?;
        self.last_used_idx = self.last_used_idx.wrapping_add(1);

        Ok(len)
    }
}

/// The inner layout of a VirtQueue.
pub struct VirtQueueLayout {
    avail_offset: usize,
    used_offset: usize,
    size: usize,
}

/// Align `size` up to a page.
fn align_up(size: usize) -> usize {
    (size + PAGE_SIZE) & !(PAGE_SIZE - 1)
}

impl VirtQueueLayout {
    pub fn new(queue_size: u16) -> Option<Self> {
        let queue_size = queue_size as usize;
        if !queue_size.is_power_of_two() || queue_size > 32768 {
            return None;
        }

        let desc = size_of::<Descriptor>() * queue_size;
        let avail = size_of::<u16>() * (3 + queue_size);
        let used = size_of::<u16>() * 3 + size_of::<UsedElem>() * queue_size;
        Some(VirtQueueLayout {
            avail_offset: desc,
            used_offset: align_up(desc + avail),
            size: align_up(desc + avail) + align_up(used),
        })
    }

    pub fn size(&self) -> usize {
        self.size
    }
}

#[repr(C, align(16))]
#[derive(Debug)]
struct Descriptor {
    addr: Volatile<u64>,
    len: Volatile<u32>,
    flags: Volatile<DescFlags>,
    next: Volatile<u16>,
}

impl Descriptor {
    fn set_buf(&mut self, buf: &VirtqueueBuf) {
        self.addr.write(buf.addr);
        self.len.write(buf.len);
    }
}

bitflags! {
    /// Descriptor flags
    struct DescFlags: u16 {
        const NEXT = 1;
        const WRITE = 2;
        const INDIRECT = 4;
    }
}

/// The driver uses the available ring to offer buffers to the device:
/// each ring entry refers to the head of a descriptor chain.
/// It is only written by the driver and read by the device.
#[repr(C)]
#[derive(Debug)]
struct AvailRing {
    flags: Volatile<u16>,
    /// A driver MUST NOT decrement the idx.
    idx: Volatile<u16>,
    ring: [Volatile<u16>; MAX_QUEUE_SIZE], // actual size: queue_size
    used_event: Volatile<u16>,             // unused
}

/// The used ring is where the device returns buffers once it is done with them:
/// it is only written to by the device, and read by the driver.
#[repr(C)]
#[derive(Debug)]
struct UsedRing {
    flags: Volatile<u16>,
    idx: Volatile<u16>,
    ring: [UsedElem; MAX_QUEUE_SIZE], // actual size: queue_size
    avail_event: Volatile<u16>,       // unused
}

#[repr(C)]
#[derive(Debug)]
struct UsedElem {
    id: Volatile<u32>,
    len: Volatile<u32>,
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_struct_size() {
        assert_eq!(size_of::<Descriptor>(), 16);
        assert_eq!(size_of::<DescFlags>(), 2);
        assert_eq!(size_of::<AvailRing>(), 70);
        assert_eq!(size_of::<UsedRing>(), 264);
        assert_eq!(size_of::<UsedElem>(), 8);
    }
}
