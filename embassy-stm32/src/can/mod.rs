//! Controller Area Network (CAN)
#![macro_use]

#[cfg_attr(can_bxcan, path = "bxcan.rs")]
#[cfg_attr(any(can_fdcan_v1, can_fdcan_h7), path = "fdcan.rs")]
mod _version;
pub use _version::*;
// #[cfg(can_bxcan)]
// mod bxcan;
// #[cfg(any(can_fdcan_v1, can_fdcan_h7))]
// mod fdcan;

// #[cfg(can_bxcan)]
// pub use bxcan::*;
// #[cfg(any(can_fdcan_v1, can_fdcan_h7))]
// pub use fdcan::*;

// use defmt::Format;

// use crate::time::Hertz;

// #[derive(Format)]
// struct BitTimings {
//     prescaler: u16,
//     tseg1: u8,
//     tseg2: u8,
//     sjw: u8,
// }

// const fn calc_bit_timings(periph_clock: Hertz, can_bitrate: u32) -> Option<BitTimings> {
//     const BS1_MAX: u8 = 16;
//     const BS2_MAX: u8 = 8;
//     const MAX_SAMPLE_POINT_PERMILL: u16 = 900;

//     let periph_clock = periph_clock.0;

//     if can_bitrate < 1000 {
//         return None;
//     }

//     // Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
//     //      CAN in Automation, 2003
//     //
//     // According to the source, optimal quanta per bit are:
//     //   Bitrate        Optimal Maximum
//     //   1000 kbps      8       10
//     //   500  kbps      16      17
//     //   250  kbps      16      17
//     //   125  kbps      16      17
//     let max_quanta_per_bit: u8 = if can_bitrate >= 1_000_000 { 10 } else { 17 };

//     // Computing (prescaler * BS):
//     //   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
//     //   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
//     // let:
//     //   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
//     //   PRESCALER_BS = PRESCALER * BS
//     // ==>
//     //   PRESCALER_BS = PCLK / BITRATE
//     let prescaler_bs = periph_clock / can_bitrate;

//     // Searching for such prescaler value so that the number of quanta per bit is highest.
//     let mut bs1_bs2_sum = max_quanta_per_bit - 1;
//     while (prescaler_bs % (1 + bs1_bs2_sum) as u32) != 0 {
//         if bs1_bs2_sum <= 2 {
//             return None; // No solution
//         }
//         bs1_bs2_sum -= 1;
//     }

//     let prescaler = prescaler_bs / (1 + bs1_bs2_sum) as u32;
//     if (prescaler < 1) || (prescaler > 1024) {
//         return None; // No solution
//     }

//     // Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
//     // We need to find such values so that the sample point is as close as possible to the optimal value,
//     // which is 87.5%, which is 7/8.
//     //
//     //   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
//     //   {{bs2 -> (1 + bs1)/7}}
//     //
//     // Hence:
//     //   bs2 = (1 + bs1) / 7
//     //   bs1 = (7 * bs1_bs2_sum - 1) / 8
//     //
//     // Sample point location can be computed as follows:
//     //   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
//     //
//     // Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
//     //   - With rounding to nearest
//     //   - With rounding to zero
//     let mut bs1 = ((7 * bs1_bs2_sum - 1) + 4) / 8; // Trying rounding to nearest first
//     let mut bs2 = bs1_bs2_sum - bs1;
//     core::assert!(bs1_bs2_sum > bs1);

//     let sample_point_permill = 1000 * ((1 + bs1) / (1 + bs1 + bs2)) as u16;
//     if sample_point_permill > MAX_SAMPLE_POINT_PERMILL {
//         // Nope, too far; now rounding to zero
//         bs1 = (7 * bs1_bs2_sum - 1) / 8;
//         bs2 = bs1_bs2_sum - bs1;
//     }

//     // Check is BS1 and BS2 are in range
//     if (bs1 < 1) || (bs1 > BS1_MAX) || (bs2 < 1) || (bs2 > BS2_MAX) {
//         return None;
//     }

//     // Check if final bitrate matches the requested
//     if can_bitrate != (periph_clock / (prescaler * (1 + bs1 + bs2) as u32)) {
//         return None;
//     }

//     // One is recommended by DS-015, CANOpen, and DeviceNet
//     let sjw = 1;

//     // Pack into BTR register values
//     // Some((sjw - 1) << 24 | (bs1 as u32 - 1) << 16 | (bs2 as u32 - 1) << 20 | (prescaler - 1))

//     Some(BitTimings {
//         prescaler: prescaler as u16,
//         tseg1: bs1,
//         tseg2: bs2,
//         sjw,
//     })
// }
