#include <ros/ros.h>
#include <mega_caretaker/MasterPacket.h>

using namespace ros;

bool waiting = true;
void block_wait_drive(const mega_caretaker::MasterPacket& msg) {
	waiting = false;
}

int main(int argc, char* argv[]) {
	init(argc, argv, "cabman");
	ROS_INFO("CABBY --> starting engines.");

	NodeHandle nh;
	Publisher pub = nh.advertise<mega_caretaker::MasterPacket>("/mega/command", 1000);
	Subscriber sub = nh.subscribe("/mega/response", 1000, &block_wait_drive);

	while(pub.getNumSubscribers() <= 0);

	ROS_INFO("CABBY --> publisher ready.");
	int cmd, pay;

	if (argc == 3) {
		cmd = atoi(argv[1]);
		pay = atoi(argv[2]);
	} else {
		ROS_ERROR("must have 2 additional arguments. see mega_caretaker/MegaPacket for more info.");
		return(70);
	}

	ROS_INFO("CABBY --> command: (%d, %d)", cmd, pay);
	mega_caretaker::MasterPacket drive;
	drive.msgType = cmd;
	drive.payload = pay;

	pub.publish(drive);
	do {
		spinOnce();
	} while(waiting);

	ROS_INFO("CABBY --> unblocked. your destination is on the left.");
	// i know, that's what i just said!
	return(0);
}
