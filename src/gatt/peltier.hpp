#pragma once

#include "ble/att_server.h"
#include <cstdint>
#include <optional>
#include <span>

namespace nevermore::gatt::peltier {

bool init();
void disconnected(hci_con_handle_t conn);
std::optional<uint16_t> attr_read(
        hci_con_handle_t conn, uint16_t attr, uint16_t offset, std::span<uint8_t> buffer);
std::optional<int> attr_write(hci_con_handle_t conn, uint16_t attr, std::span<uint8_t const> buffer);

}  // namespace nevermore::gatt::peltier