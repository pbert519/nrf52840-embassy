//! Stores log data on a external flash in a ring buffer like structure
//! ToDo: Overwrite old entries if buffer is full
//! ToDo: Find and use old entries after power cycle

use bincode::{config, error::DecodeError, error::EncodeError, Decode, Encode};
use embedded_storage::nor_flash::NorFlash;

const PAGE_SIZE: usize = crate::board_config::EXTERNAL_FLASH_PAGE_SIZE;

#[derive(Debug)]
pub enum Error {
    Decode,
    Encode,
    StorageFull,
    InvalidIndex,
}

impl From<DecodeError> for Error {
    fn from(_: DecodeError) -> Self {
        Error::Decode
    }
}
impl From<EncodeError> for Error {
    fn from(_: EncodeError) -> Self {
        Error::Encode
    }
}

#[repr(C, align(4))]
struct AlignedBuf([u8; PAGE_SIZE]);

#[derive(Encode, Decode, PartialEq, Debug, defmt::Format)]
struct LogEntry<T: bincode::Decode + bincode::Encode + defmt::Format> {
    id: usize,
    version: usize,
    data: T,
}

pub struct LogStorage<StorageProvider, Data>
where
    StorageProvider: NorFlash,
    Data: bincode::Encode + bincode::Decode,
{
    _type: core::marker::PhantomData<Data>,
    storage: StorageProvider,
    nbr_of_pages: usize,
    head: usize,
}

impl<StorageProvider, Data> LogStorage<StorageProvider, Data>
where
    StorageProvider: NorFlash,
    Data: bincode::Encode + bincode::Decode + defmt::Format,
{
    pub fn new(storage: StorageProvider) -> Self {
        // Todo read the flash to detect already written logs
        let nbr_of_pages = storage.capacity() / PAGE_SIZE;

        let mut tmp = Self {
            storage,
            nbr_of_pages,
            head: 0,
            _type: core::marker::PhantomData,
        };
        // try to synchronize with flash
        // if not already written data was found
        // erase flash
        if tmp.sync_flash().is_err() {
            tmp.erase_all();
            defmt::debug!("No previous flash pages found!.");
        } else {
            defmt::debug!("Found {} entries in flash", tmp.head);
        }
        tmp
    }

    fn sync_flash(&mut self) -> Result<(), ()> {
        while self.head < self.nbr_of_pages {
            if let Ok(entry) = self.at_unchecked(self.head) {
                if entry.id != self.head || entry.version != 0x01 {
                    // not a valid entry
                    break;
                }
            } else {
                break;
            }
            self.head += 1;
        }

        if self.head > 0 {
            // at least one valid entry found
            Ok(())
        } else {
            // no valid entries found
            Err(())
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn len(&self) -> usize {
        self.head
    }

    fn at_unchecked(&mut self, index: usize) -> Result<LogEntry<Data>, Error> {
        let address: u32 = (index * PAGE_SIZE) as u32;
        let mut bin = AlignedBuf([0xFFu8; PAGE_SIZE]);
        self.storage.read(address, &mut bin.0).unwrap();
        let (entry, _): (LogEntry<Data>, _) =
            bincode::decode_from_slice(&bin.0, config::standard())?;
        Ok(entry)
    }

    pub fn at(&mut self, index: usize) -> Result<Data, Error> {
        if !self.is_empty() || index >= self.head {
            Ok(self.at_unchecked(index)?.data)
        } else {
            Err(Error::InvalidIndex)
        }
    }

    pub fn add_entry(&mut self, data: Data) -> Result<(), Error> {
        if self.len() == self.nbr_of_pages {
            Err(Error::StorageFull)
        } else {
            let address: u32 = (self.head * PAGE_SIZE) as u32;
            // convert T to buffer and write to flash
            let entry = LogEntry {
                id: self.head,
                version: 0x01,
                data,
            };
            let mut bin = AlignedBuf([0xFFu8; PAGE_SIZE]);
            let bytes = bincode::encode_into_slice(entry, &mut bin.0, config::standard())?;
            let bytes = (bytes + 3usize) & !0x03;
            self.storage.write(address, &bin.0[..bytes]).unwrap();
            self.head += 1;
            Ok(())
        }
    }

    pub fn erase_all(&mut self) {
        // erase memory buffer
        let last_address = self.storage.capacity() as u32;
        self.storage.erase(0, last_address).unwrap();
        self.head = 0;
    }
}
