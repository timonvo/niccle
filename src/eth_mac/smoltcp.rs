use crate::eth_mac;
use log::debug;

/// An implementation of smoltcp's `Device` trait, built on top of our `eth_mac` MAC layer.
pub struct SmolMac<'inner_mac, P: eth_mac::PhyTx<'inner_mac>, const BUFFER_CAP: usize> {
    tx: &'inner_mac mut eth_mac::MacTx<'inner_mac, P>,
    rx: &'inner_mac eth_mac::MacRx<BUFFER_CAP>,
}

impl<'inner_mac, P: eth_mac::PhyTx<'inner_mac>, const BUFFER_CAP: usize>
    SmolMac<'inner_mac, P, BUFFER_CAP>
{
    pub fn new(
        tx: &'inner_mac mut eth_mac::MacTx<'inner_mac, P>,
        rx: &'inner_mac eth_mac::MacRx<BUFFER_CAP>,
    ) -> SmolMac<'inner_mac, P, BUFFER_CAP> {
        SmolMac { tx, rx }
    }
}

impl<'inner_mac, P: eth_mac::PhyTx<'inner_mac>, const BUFFER_CAP: usize> smoltcp::phy::Device
    for SmolMac<'inner_mac, P, BUFFER_CAP>
{
    type RxToken<'a> = SmolRxToken<'a> where Self: 'a;
    type TxToken<'a> = SmolTxToken<'a, 'inner_mac, P> where Self: 'a;

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        self.rx
            .receive()
            .map(|frame| (SmolRxToken { frame }, SmolTxToken { mac_tx: self.tx }))
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        Some(SmolTxToken { mac_tx: self.tx })
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.medium = smoltcp::phy::Medium::Ethernet;
        caps.max_transmission_unit = eth_mac::MAX_FRAME_SIZE - eth_mac::FCS_SIZE;
        // This determines the TCP window size that smoltcp will advertise, and should be set
        // to the maximum number of packets we can buffer in the 'device'. If the number of
        // packets we can buffer in the device is much smaller than the number of packets that
        // would fit in the TCP device buffer, then smoltcp will use the smaller of the two
        // capacities as the advertise TCP receive window size.
        caps.max_burst_size = Some(BUFFER_CAP);
        caps
    }
}

pub struct SmolRxToken<'a> {
    frame: eth_mac::FrameRef<'a>,
}

impl<'a> smoltcp::phy::RxToken for SmolRxToken<'a> {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = f(self.frame.data());
        debug!("SmolTcp RX consume called");
        result
    }
}

pub struct SmolTxToken<'a, 'inner_mac, P: eth_mac::PhyTx<'inner_mac>> {
    mac_tx: &'a mut eth_mac::MacTx<'inner_mac, P>,
}

impl<'a, 'inner_mac, P: eth_mac::PhyTx<'inner_mac>> smoltcp::phy::TxToken
    for SmolTxToken<'a, 'inner_mac, P>
{
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = self.mac_tx.transmit_frame_with(len, f);
        debug!("SmolTcp TX consume called");
        result
    }
}
