// Copyright (c) Microsoft Corporation
//
// SPDX-License-Identifier: BSD-2-Clause-Patent

use interrupt_emu as intr;
pub use intr::InterruptStack;

#[derive(Copy, Clone)]
pub struct InterruptCallback(fn(&mut InterruptStack));

impl InterruptCallback {
    pub fn new(cb: fn(&mut InterruptStack)) -> Self {
        Self(cb)
    }
    pub fn call(&self, stack: &mut InterruptStack) {
        (self.0)(stack)
    }
}

pub fn register_interrupt_callback(vector: usize, cb: InterruptCallback) -> Result<(), ()> {
    // Validate vector fits in u8 (IDT has 256 entries)
    if vector >= 256 {
        return Err(());
    }
    // Store raw fn into interrupt-emu registry
    intr::register(vector as u8, cb.0);
    Ok(())
}
