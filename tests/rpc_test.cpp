#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>
#include "<filename>.pb.h"
#include "<filename>.grpc.pb.h"

class BeagleboneServiceImpl final : public Beaglebone::Service {
public:
  grpc::Status Beagle(grpc::ServerContext* context, const BeagleRequest* request,
                      Logging_Packet* response) override {
    // Process the Jetson_to_beagle message received from the client
    const Jetson_to_beagle& jetson_msg = request->jetson_to_beagle();
    std::cout << "Received keyboard_cmd: " << jetson_msg.keyboard_cmd() << std::endl;
    for (float xf : jetson_msg.xf()) {
      std::cout << "Received xf: " << xf << std::endl;
    }
    
    // Create and fill in a Logging_Packet message to send back to the client
    Logging* log = response->add_logs();
    log->set_time(123.45);
    // Set other fields as needed

    return grpc::Status::OK;
  }
};

void RunServer() {
  std::string server_address("0.0.0.0:50051");
  BeagleboneServiceImpl service;

  grpc::ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}

int main(int argc, char** argv) {
  RunServer();
  return 0;
}

