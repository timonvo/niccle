/// An implementation of smoltcp's `Device` trait, built on top of our `eth_mac` MAC layer.
use crate::eth_mac;
use log::debug;

pub struct SmolMac<'inner_mac, P: eth_mac::PhyTx<'inner_mac>, const BUFFER_CAP: usize> {
    tx: &'inner_mac mut eth_mac::MacTx<'inner_mac, P>,
    rx: eth_mac::MacRxPacketConsumer<'inner_mac, BUFFER_CAP>,
}

impl<'inner_mac, P: eth_mac::PhyTx<'inner_mac>, const BUFFER_CAP: usize>
    SmolMac<'inner_mac, P, BUFFER_CAP>
{
    pub fn new(
        tx: &'inner_mac mut eth_mac::MacTx<'inner_mac, P>,
        rx: eth_mac::MacRxPacketConsumer<'inner_mac, BUFFER_CAP>,
    ) -> SmolMac<'inner_mac, P, BUFFER_CAP> {
        SmolMac { tx, rx }
    }
}

impl<'inner_mac, P: eth_mac::PhyTx<'inner_mac>, const BUFFER_CAP: usize> smoltcp::phy::Device
    for SmolMac<'inner_mac, P, BUFFER_CAP>
{
    type RxToken<'a> = SmolRxToken<'a, BUFFER_CAP> where Self: 'a;
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
        // This determines the TCP window size that smoltcp will advertise (if `max_burst_size *
        // MTU` is smaller than the actual TCP RX buffer size). I've found it easier to achieve
        // maximum throughput by manually tuning the device and TCP RX buffer sizes, however.
        caps.max_burst_size = None;
        caps
    }
}

pub struct SmolRxToken<'a, const BUFFER_CAP: usize> {
    frame: eth_mac::FrameRef<'a, BUFFER_CAP>,
}

impl<'a, const BUFFER_CAP: usize> smoltcp::phy::RxToken for SmolRxToken<'a, BUFFER_CAP> {
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
