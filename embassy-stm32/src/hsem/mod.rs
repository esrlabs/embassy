//! Hardware Semaphore (HSEM)

// TODO: This code works for all HSEM implemenations except for the STM32WBA52/4/5xx MCUs.
// Those MCUs have a different HSEM implementation (Secure semaphore lock support,
// Privileged / unprivileged semaphore lock support, Semaphore lock protection via semaphore attribute),
// which is not yet supported by this code.
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use crate::rcc::RccPeripheral;
use crate::{interrupt, pac, peripherals, Peripheral};
use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

static HSEM_WAKER: AtomicWaker = AtomicWaker::new();

/// HSEM error.
#[derive(Debug)]
pub enum HsemError {
    /// Locking the semaphore failed.
    LockFailed,
}

/// HSEM interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let c = match get_current_coreid() {
            CoreId::Core0 => 0,
            CoreId::Core1 => 1,
        };

        // Get the list of masked freed semaphores
        let statusreg = T::regs().misr(c).read().0;
        // Disable Interrupts
        T::regs().ier(c).modify(|w| w.0 = w.0 & !statusreg);
        // Clear Flags
        T::regs().icr(c).modify(|w| w.0 = statusreg);

        HSEM_WAKER.wake();
    }
}

/// CPU core.
/// The enum values are identical to the bus master IDs / core Ids defined for each
/// chip family (i.e. stm32h747 see rm0399 table 95)
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[repr(u8)]
#[derive(defmt::Format)]
pub enum CoreId {
    #[cfg(any(stm32h745, stm32h747, stm32h755, stm32h757))]
    /// Cortex-M7, core 1.
    Core0 = 0x3,

    #[cfg(any(stm32h745, stm32h747, stm32h755, stm32h757))]
    /// Cortex-M4, core 2.
    Core1 = 0x1,

    #[cfg(not(any(stm32h745, stm32h747, stm32h755, stm32h757)))]
    /// Cortex-M4, core 1
    Core0 = 0x4,

    #[cfg(any(stm32wb, stm32wl))]
    // Cortex-M0+, core 2.
    Core1 = 0x8,
}

/// Get the current core id
/// This code assume that it is only executed on a Cortex-M M0+, M4 or M7 core.
#[inline(always)]
pub fn get_current_coreid() -> CoreId {
    let cpuid = unsafe { cortex_m::peripheral::CPUID::PTR.read_volatile().base.read() };
    match (cpuid & 0x000000F0) >> 4 {
        #[cfg(any(stm32wb, stm32wl))]
        0x0 => CoreId::Core1,

        #[cfg(not(any(stm32h745, stm32h747, stm32h755, stm32h757)))]
        0x4 => CoreId::Core0,

        #[cfg(any(stm32h745, stm32h747, stm32h755, stm32h757))]
        0x4 => CoreId::Core1,

        #[cfg(any(stm32h745, stm32h747, stm32h755, stm32h757))]
        0x7 => CoreId::Core0,
        _ => panic!("Unknown Cortex-M core"),
    }
}

/// Translates the core ID to an index into the interrupt registers.
#[inline(always)]
fn core_id_to_index(core: CoreId) -> usize {
    match core {
        CoreId::Core0 => 0,
        CoreId::Core1 => 1,
    }
}

/// HSEM driver
pub struct HardwareSemaphore<'d, T: Instance> {
    _inner: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> HardwareSemaphore<'d, T> {
    /// Creates a new HardwareSemaphore instance.
    pub fn new(
        peripheral: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        into_ref!(peripheral);
        HardwareSemaphore { _inner: peripheral }
    }

    /// Locks the semaphore.
    /// The 2-step lock procedure consists in a write to lock the semaphore, followed by a read to
    /// check if the lock has been successful, carried out from the HSEM_Rx register.
    pub fn two_step_lock(&mut self, sem_id: u8, process_id: u8) -> Result<(), HsemError> {
        T::regs().r(sem_id as usize).write(|w| {
            w.set_procid(process_id);
            w.set_coreid(get_current_coreid() as u8);
            w.set_lock(true);
        });
        let reg = T::regs().r(sem_id as usize).read();
        match (
            reg.lock(),
            reg.coreid() == get_current_coreid() as u8,
            reg.procid() == process_id,
        ) {
            (true, true, true) => Ok(()),
            _ => Err(HsemError::LockFailed),
        }
    }

    /// Locks the semaphore.
    /// The 1-step procedure consists in a read to lock and check the semaphore in a single step,
    /// carried out from the HSEM_RLRx register.
    pub fn one_step_lock(&mut self, sem_id: u8) -> bool {
        let reg = T::regs().rlr(sem_id as usize).read();
        match (reg.lock(), reg.coreid() == get_current_coreid() as u8, reg.procid()) {
            (true, true, 0) => true,
            _ => false,
        }
    }

    /// Locks the semaphore. If the semaphore is already locked,
    /// the function will return a future
    pub async fn lock(&mut self, sem_id: u8) -> bool {
        if !self.is_semaphore_locked(sem_id) {
            return self.one_step_lock(sem_id);
        }

        self.enable_interrupt(get_current_coreid(), sem_id, true);

        poll_fn(|cx| {
            HSEM_WAKER.register(cx.waker());
            if !self.is_semaphore_locked(sem_id) {
                Poll::Ready(self.one_step_lock(sem_id))
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Unlocks the semaphore.
    /// Unlocking a semaphore is a protected process, to prevent accidental clearing by a AHB bus
    /// core ID or by a process not having the semaphore lock right.
    pub fn unlock(&mut self, sem_id: u8, process_id: u8) {
        T::regs().r(sem_id as usize).write(|w| {
            w.set_procid(process_id);
            w.set_coreid(get_current_coreid() as u8);
            w.set_lock(false);
        });
    }

    /// Unlocks all semaphores.
    /// All semaphores locked by a COREID can be unlocked at once by using the HSEM_CR
    /// register. Write COREID and correct KEY value in HSEM_CR. All locked semaphores with a
    /// matching COREID are unlocked, and may generate an interrupt when enabled.
    pub fn unlock_all(&mut self, key: u16, core_id: u8) {
        T::regs().cr().write(|w| {
            w.set_key(key);
            w.set_coreid(core_id);
        });
    }

    /// Checks if the semaphore is locked.
    pub fn is_semaphore_locked(&self, sem_id: u8) -> bool {
        T::regs().r(sem_id as usize).read().lock()
    }

    /// Sets the clear (unlock) key
    pub fn set_clear_key(&mut self, key: u16) {
        T::regs().keyr().modify(|w| w.set_key(key));
    }

    /// Gets the clear (unlock) key
    pub fn get_clear_key(&mut self) -> u16 {
        T::regs().keyr().read().key()
    }

    /// Sets the interrupt enable bit for the semaphore.
    pub fn enable_interrupt(&mut self, core_id: CoreId, sem_x: u8, enable: bool) {
        T::regs()
            .ier(core_id_to_index(core_id))
            .modify(|w| w.set_ise(sem_x as usize, enable));
    }

    /// Gets the interrupt enable bit for the semaphore.
    pub fn is_interrupt_enabled(&mut self, core_id: CoreId, sem_x: u8) -> bool {
        T::regs().ier(core_id_to_index(core_id)).read().ise(sem_x as usize)
    }

    /// Clears the interrupt flag for the semaphore.
    pub fn clear_interrupt(&mut self, core_id: CoreId, sem_x: u8) {
        T::regs()
            .icr(core_id_to_index(core_id))
            .write(|w| w.set_isc(sem_x as usize, false));
    }

    /// Enables interrupts for the semaphore and waits for the semaphore to be unlocked.
    /// In case the semaphore is already unlocked, the function returns immediately and
    /// does not enable the interrupt.
    pub async fn wait_unlocked(&mut self, sem_id: u8) -> Result<(), HsemError> {
        if !self.is_semaphore_locked(sem_id) {
            return Ok(());
        }

        self.enable_interrupt(get_current_coreid(), sem_id, true);

        poll_fn(|cx| {
            HSEM_WAKER.register(cx.waker());
            if !self.is_semaphore_locked(sem_id) {
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

trait SealedInstance {
    fn regs() -> pac::hsem::Hsem;
}

/// HSEM instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + Peripheral<P = Self> + RccPeripheral + Send + 'static {
    /// Interrupt for this HSEM instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

/// HSEM peripheral instances.
impl Instance for peripherals::HSEM {
    // TODO: add support for more chips
    // maybe move to identification of the core for which the code
    // gets compiled into the build.rs
    #[cfg(any(feature = "stm32h747zi-cm7", feature = "stm32h755zi-cm7"))]
    type Interrupt = crate::interrupt::typelevel::HSEM1;

    #[cfg(any(feature = "stm32h747zi-cm4", feature = "stm32h755zi-cm4"))]
    type Interrupt = crate::interrupt::typelevel::HSEM2;
}

impl SealedInstance for peripherals::HSEM {
    fn regs() -> crate::pac::hsem::Hsem {
        crate::pac::HSEM
    }
}
