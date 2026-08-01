#include <cassert>
#include <iostream>

#include "n203/radar/frame_validation.hpp"

void runDisplayAppTests();

int main() {
    n203::radar::RadarFrame valid{};
    valid.capabilities.flags = n203::radar::kPosition | n203::radar::kRange;
    valid.contact_count = 1;
    valid.contacts[0].id = 3;
    valid.contacts[0].valid_fields = n203::radar::kContactPosition | n203::radar::kContactRange;
    assert(n203::radar::validateFrame(valid) == n203::radar::FrameValidationError::kNone);

    valid.contact_count = 2;
    valid.contacts[1] = valid.contacts[0];
    assert(n203::radar::validateFrame(valid) == n203::radar::FrameValidationError::kDuplicateContactId);

    valid.contact_count = static_cast<std::uint8_t>(n203::radar::kMaximumContacts + 1U);
    assert(n203::radar::validateFrame(valid) == n203::radar::FrameValidationError::kTooManyContacts);

    runDisplayAppTests();
    std::cout << "N203 unit tests passed.\n";
    return 0;
}
