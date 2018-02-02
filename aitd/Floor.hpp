#pragma once

#include "Room.hpp"
#include "RoomCamera.hpp"
#include <map>
namespace aitd {
class Floor {

public:

	typedef std::shared_ptr<Floor> Ptr;
	
	void load(int floor_idx);

	RoomCamera::Ptr getCamera(int cam_idx) {
		assert(camera_map.count(cam_idx));
		return camera_map[cam_idx];
	}

	Room::Ptr getRoom(int room_idx) {
		assert(room_idx < room_vector.size());
		return room_vector[room_idx];
	}

	std::vector<Room::Ptr> getRoomVector() {
		return room_vector;
	}
	
protected:

	std::vector<Room::Ptr> room_vector;
	std::map<int, RoomCamera::Ptr> camera_map;
};

}
