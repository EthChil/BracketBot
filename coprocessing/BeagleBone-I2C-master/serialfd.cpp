#include <iostream>
#include <fstream>
#include <string>

int main() {
    // Open the file.
    std::fstream fdcanusb("/dev/fdcanusb");

    // Send command.
    {
        std::string cmd = "can send 8001 420120\n";
        fdcanusb.write(cmd.data(), cmd.size());
    }

    // Read and ignore OK.
    {
        std::string line;
        std::getline(fdcanusb, line);
    }

    // Read response.
    std::string response;
    std::getline(fdcanusb, response);

    // Output the response.
    std::cout << "Response: " << response << std::endl;

    // NOTE! If the board doesn't respond, the above simple strategy will never return.

    return 0;
}
