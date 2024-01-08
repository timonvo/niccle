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
        "PHY RX: Packets:  {:5}, trunc:       {:4}, LTPs:        {:7}",
        phy_stats.rx.probable_packets_received,
        phy_stats.rx.truncated_packets_received,
        phy_stats.rx.ltps_received,
    );

    info!(
        "MAC RX: Packets:  {:5}, invalid SFD: {:4}, invalid CRC: {:7}",
        mac_rx_stats.valid_frames_received,
        mac_rx_stats.invalid_sfd_packets_received,
        mac_rx_stats.invalid_crc_packets_received,
    );
    info!(
        "        Consumed: {:5}, dropped:     {:4}",
        mac_rx_stats.valid_frames_consumed, mac_rx_stats.valid_frames_dropped
    );
}
