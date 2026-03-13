use defmt::error;
use embassy_futures::select::{Either, select};
use embassy_stm32::can::{
    Can, Frame,
    frame::{FdFrame, Header},
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embedded_can::{ExtendedId, Id, StandardId};

// channels
pub const CAN_RX_CHANNEL_SIZE: usize = 8;
static CAN_RX_CHANNEL: Channel<ThreadModeRawMutex, FdFrame, CAN_RX_CHANNEL_SIZE> = Channel::new();

pub fn get_can_rx_receiver() -> Receiver<'static, ThreadModeRawMutex, FdFrame, CAN_RX_CHANNEL_SIZE>
{
    CAN_RX_CHANNEL.receiver()
}

pub enum CanRequest {
    Tx { id: Id, len: u8, data: [u8; 8] },
}

pub const CAN_REQUEST_CHANNEL_SIZE: usize = 4;
static CAN_TX_CHANNEL: Channel<ThreadModeRawMutex, CanRequest, CAN_REQUEST_CHANNEL_SIZE> =
    Channel::new();

pub fn get_can_req_sender()
-> Sender<'static, ThreadModeRawMutex, CanRequest, CAN_REQUEST_CHANNEL_SIZE> {
    CAN_TX_CHANNEL.sender()
}

#[embassy_executor::task]
pub async fn can_handler(mut can: Can<'static>) {
    let rx_sender = CAN_RX_CHANNEL.sender();
    let tx_receiver = CAN_TX_CHANNEL.receiver();
    loop {
        match select(can.read_fd(), tx_receiver.receive()).await {
            Either::First(read_result) => match read_result {
                Ok(envelope) => {
                    rx_sender.send(envelope.frame).await;
                }
                Err(err) => {
                    error!("Error in frame: {:?}", err);
                    break;
                }
            },
            Either::Second(request) => match request {
                CanRequest::Tx { id, len, data } => {
                    let header = Header::new(id, len, false);
                    let frame = Frame::new(header, &data).unwrap();
                    _ = can.write(&frame).await;
                }
            },
        }
    }
}

pub fn make_tx_std_request(id: u16, d: &[u8]) -> CanRequest {
    let mut data = [0u8; 8];
    data[..d.len()].copy_from_slice(d);
    CanRequest::Tx {
        id: Id::Standard(StandardId::new(id).unwrap()),
        len: d.len() as u8,
        data,
    }
}

pub fn make_tx_ext_request(id: u32, d: &[u8]) -> CanRequest {
    let mut data = [0u8; 8];
    data[..d.len()].copy_from_slice(d);
    CanRequest::Tx {
        id: Id::Extended(ExtendedId::new(id).unwrap()),
        len: d.len() as u8,
        data,
    }
}

/*
pub fn make_extended_frame(id: u32, data: &[u8]) -> FdFrame {
    let header = Header::new_fd(
        Id::Extended(ExtendedId::new(id).unwrap()),
        data.len() as u8,
        false,
        false,
    );
    FdFrame::new(header, &data).unwrap()
}

pub fn make_standard_frame(id: u16, data: &[u8]) -> FdFrame {
    let header = Header::new_fd(
        Id::Standard(StandardId::new(id).unwrap()),
        data.len() as u8,
        false,
        false,
    );
    FdFrame::new(header, &data).unwrap()
}
*/
