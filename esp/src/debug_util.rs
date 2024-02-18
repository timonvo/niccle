use log::info;
use niccle::eth_mac;

use crate::eth_phy;

pub fn log_phy_mac_stats(phy_stats: &eth_phy::TxRxStats, mac_rx_stats: &eth_mac::RxStats) {
    info!("--- Stats ---");
    info!(
        "PHY TX: Packets:  {:5},              {:4}  LTPs:        {:7}",
        phy_stats.tx.packets_sent, "", phy_stats.tx.ltps_sent,
    );
    info!(
        "PHY RX: Packets:  {:5}, trunc:       {:4}, LTPs:        {:7}, OOM: {:3}",
        phy_stats.rx.probable_packets_received,
        phy_stats.rx.truncated_packets_received,
        phy_stats.rx.ltps_received,
        phy_stats.rx.out_of_buffer_space_events,
    );

    info!(
        "MAC RX: Packets:  {:5}, invalid SFD: {:4}, invalid CRC: {:7}",
        mac_rx_stats.valid_frames_received,
        mac_rx_stats.invalid_sfd_packets_received,
        mac_rx_stats.invalid_crc_packets_received,
    );
    info!("          Valid:  {:5}", mac_rx_stats.valid_frames_received);
}
