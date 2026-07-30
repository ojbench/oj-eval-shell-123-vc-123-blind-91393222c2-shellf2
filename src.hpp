#include "fstream.h"
#include <algorithm>
#include <cstring>
#include <vector>

// Disk event types: normal, failure, replacement.
enum class EventType {
  NORMAL,
  FAILED,
  REPLACED
};

class RAID5Controller {
private:
  std::vector<sjtu::fstream *> drives_;
  int blocks_per_drive_;
  int block_size_;
  int num_disks_;
  int failed_drive_ = -1;

  int parity_drive_for_stripe(int stripe) const {
    return num_disks_ - 1 - (stripe % num_disks_);
  }

  int stripe_of_block(int block_id) const { return block_id / (num_disks_ - 1); }

  int offset_in_stripe(int block_id) const { return block_id % (num_disks_ - 1); }

  int physical_drive_for_block(int block_id) const {
    const int stripe = stripe_of_block(block_id);
    const int parity = parity_drive_for_stripe(stripe);
    const int pos = offset_in_stripe(block_id);
    return pos < parity ? pos : pos + 1;
  }

  void seek_block(sjtu::fstream &file, int block_index) {
    file.clear();
    const std::streamoff off = static_cast<std::streamoff>(block_index) * block_size_;
    file.seekg(off, std::ios::beg);
    file.seekp(off, std::ios::beg);
  }

  void read_raw(int drive_id, int block_index, char *buf) {
    sjtu::fstream &file = *drives_[drive_id];
    seek_block(file, block_index);
    file.read(buf, block_size_);
  }

  void write_raw(int drive_id, int block_index, const char *buf) {
    sjtu::fstream &file = *drives_[drive_id];
    seek_block(file, block_index);
    file.write(buf, block_size_);
    file.flush();
  }

  void reconstruct_drive(int drive_id) {
    std::vector<char> acc(block_size_);
    std::vector<char> tmp(block_size_);
    for (int stripe = 0; stripe < blocks_per_drive_; ++stripe) {
      std::fill(acc.begin(), acc.end(), 0);
      for (int d = 0; d < num_disks_; ++d) {
        if (d == drive_id) continue;
        read_raw(d, stripe, tmp.data());
        for (int i = 0; i < block_size_; ++i) acc[i] ^= tmp[i];
      }
      write_raw(drive_id, stripe, acc.data());
    }
  }

public:
  RAID5Controller(std::vector<sjtu::fstream *> drives, int blocks_per_drive,
                  int block_size = 4096)
      : drives_(std::move(drives)), blocks_per_drive_(blocks_per_drive),
        block_size_(block_size), num_disks_(static_cast<int>(drives_.size())) {}

  void Start(EventType event_type_, int drive_id) {
    if (event_type_ == EventType::NORMAL) {
      failed_drive_ = -1;
      return;
    }
    if (event_type_ == EventType::FAILED) {
      failed_drive_ = drive_id;
      return;
    }
    if (event_type_ == EventType::REPLACED) {
      reconstruct_drive(drive_id);
      if (failed_drive_ == drive_id) failed_drive_ = -1;
    }
  }

  void Shutdown() {
    for (auto *drive : drives_) {
      if (drive && drive->is_open()) {
        drive->flush();
        drive->close();
      }
    }
  }

  void ReadBlock(int block_id, char *result) {
    const int stripe = stripe_of_block(block_id);
    const int physical_drive = physical_drive_for_block(block_id);
    if (physical_drive != failed_drive_) {
      read_raw(physical_drive, stripe, result);
      return;
    }

    std::vector<char> tmp(block_size_);
    std::fill(result, result + block_size_, 0);
    for (int d = 0; d < num_disks_; ++d) {
      if (d == failed_drive_) continue;
      read_raw(d, stripe, tmp.data());
      for (int i = 0; i < block_size_; ++i) result[i] ^= tmp[i];
    }
  }

  void WriteBlock(int block_id, const char *data) {
    const int stripe = stripe_of_block(block_id);
    const int parity_drive = parity_drive_for_stripe(stripe);
    const int physical_drive = physical_drive_for_block(block_id);

    if (failed_drive_ == -1) {
      std::vector<char> old_data(block_size_);
      std::vector<char> old_parity(block_size_);
      std::vector<char> new_parity(block_size_);
      read_raw(physical_drive, stripe, old_data.data());
      read_raw(parity_drive, stripe, old_parity.data());
      for (int i = 0; i < block_size_; ++i) {
        new_parity[i] = old_parity[i] ^ old_data[i] ^ data[i];
      }
      write_raw(physical_drive, stripe, data);
      write_raw(parity_drive, stripe, new_parity.data());
      return;
    }

    if (physical_drive == failed_drive_) {
      // Recompute the missing data block and update the healthy parity block.
      std::vector<char> current(block_size_);
      std::vector<char> tmp(block_size_);
      read_raw(parity_drive, stripe, current.data());
      for (int d = 0; d < num_disks_; ++d) {
        if (d == failed_drive_) continue;
        read_raw(d, stripe, tmp.data());
        for (int i = 0; i < block_size_; ++i) current[i] ^= tmp[i];
      }
      std::vector<char> new_parity(block_size_);
      for (int i = 0; i < block_size_; ++i) {
        new_parity[i] = current[i] ^ data[i];
      }
      write_raw(parity_drive, stripe, new_parity.data());
      return;
    }

    if (parity_drive == failed_drive_) {
      // Parity is unavailable, so persist the data block only.
      write_raw(physical_drive, stripe, data);
      return;
    }

    std::vector<char> old_data(block_size_);
    std::vector<char> old_parity(block_size_);
    std::vector<char> new_parity(block_size_);
    read_raw(physical_drive, stripe, old_data.data());
    read_raw(parity_drive, stripe, old_parity.data());
    for (int i = 0; i < block_size_; ++i) {
      new_parity[i] = old_parity[i] ^ old_data[i] ^ data[i];
    }
    write_raw(physical_drive, stripe, data);
    write_raw(parity_drive, stripe, new_parity.data());
  }

  int Capacity() {
    return (num_disks_ - 1) * blocks_per_drive_;
  }
};
