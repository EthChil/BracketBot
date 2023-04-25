import grpc
import beagle_pb2 as pb2

def run():
    # Create a connection to the server
    channel = grpc.insecure_channel('localhost:50051')
    stub = pb2.BeagleboneStub(channel)

    # Create a Jetson_to_beagle message
    jetson_msg = pb2.Jetson_to_beagle()
    jetson_msg.keyboard_cmd = 123
    jetson_msg.xf.extend([1.0, 2.0, 3.0])

    # Create a request with the Jetson_to_beagle message
    request = pb2.BeagleRequest()
    request.jetson_to_beagle.CopyFrom(jetson_msg)

    # Send the request to the server
    response = stub.Beagle(request)

    # Print the Logging_Packet message received from the server
    for log in response.logs:
        print("Received log with time", log.time)
        # Process other fields here as needed

if __name__ == '__main__':
    run()