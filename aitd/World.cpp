#include "World.hpp"
#include "Components.hpp"
#include "ActorLoader.hpp"
#include <graphics/RenderSystem.hpp>

using namespace Components;

void World::load() {

	loadFloor(1);
}

//TODO: shall we preload the floors??
void World::loadFloor(int floor_id) {

	//remove entities in previous floor?
	
	//load floor
	Floor::Ptr floor_data = Floor::Ptr(new Floor());
	floor_data->load(floor_id);

	current_floor_id = floor_id;
	current_room_id = 1;
	int camera_index = 6;

	//create entities in this floor
	//Cameras ==================================================================

	std::vector<int> camera_indices = floor_data->getRoom(current_room_id)->camera_indices;

	assert(camera_index < camera_indices.size());
	RoomCamera::Ptr room_cam = floor_data->getCamera(camera_indices[camera_index]);

	//TODO: refactor in createCameraEntity()
	//NOTE: The camera coordinates are expressed wrt the room (We can consider that is "attached" to
	//the room node. We temporarily alter the location but in practice we should have a proper
	// scene-tree)
	Vec3f room_world = floor_data->getRoom(current_room_id)->world_pos.cast<float>();
	Vec3f cam_pos = room_cam->transform.col(3).head(3)/10;
	cam_pos(0) = (cam_pos(0) - room_world(0));
	cam_pos(1) = (room_world(1) - cam_pos(1));
	cam_pos(2) = (room_world(2) - cam_pos(2));
 	room_cam->transform.col(3).head(3) = cam_pos * 10;
	////////////////////////////////////////////////////////////////////////////////////////////////
	
	Entity camera = entity_manager->createLocal();
	
	// view matrix is the inverse of the camera trasnformation matrix;
	entity_manager->assign<CameraComponent>(camera.id(), room_cam->projection,
											room_cam->transform.inverse()); 
	entity_manager->assign<BgImageComponent>(camera.id(), room_cam->getBackgroundImagePtr());
	current_camera_id = camera.id();


	
	//maybe instead keep a map id/Entity

	//Objects ==================================================================
	for (auto object_it : ObjectManager::object_map) {
		if (object_it.second.stage == floor_id) {
			createObjectEntities(object_it.second);
		}
	}

	// DEBUG STUFF =============================================================
	// Display debug meshes with collison stuff
	for (auto &box : floor_data->getRoom(current_room_id)->colision_vector) {
		//Add debug objects for each colision vector
		Entity debug_obj = entity_manager->createLocal();
		entity_manager->assign<DebugComponent>(
			debug_obj.id(),
			Geometry::DebugMesh::Ptr(new Geometry::DebugBox(box->p1.cast<float>(),
															box->p2.cast<float>()))
			);
	}

	// Display location of camera
	for (auto &cam_idx : floor_data->getRoom(current_room_id)->camera_indices) {
		Room::Ptr room = floor_data->getRoom(current_room_id);		
		RoomCamera::Ptr cam = floor_data->getCamera(cam_idx);
		Vec3f cam_pos = Vec3f(cam->position(0), cam->position(1), cam->position(2));		

		// Display also a vector of direction		
		Entity debug_cam2 = entity_manager->createLocal();
		entity_manager->assign<DebugComponent>(
		debug_cam2.id(),
		Geometry::DebugMesh::Ptr(new Geometry::DebugAxis(cam->transform)));
	}
}

void World::createObjectEntities(const ObjectData& object) {

	//Create all entities at this floor
	Entity object_entity = entity_manager->createLocal();
	entity_manager->assign<TransformComponent>(
		object_entity.id(),
		object.x,
		-object.y,
		object.z,
		getRotationMatrixFromRotIndices(object.alpha,
										object.beta,
										object.gamma)
		);

	if (object.body != -1) {	
	
		//check if actor is created (create it otherwise)?
		Actor::Ptr actor_data = ActorLoader::load(object.body);		
		entity_manager->assign<MeshComponent>(object_entity.id(), actor_data->getMesh());
	}
	
}

void World::render(float dt) {

	auto camera = entity_manager->getComponentPtr<CameraComponent>(current_camera_id);
	auto bg_image = entity_manager->getComponentPtr<BgImageComponent>(current_camera_id);

	camera->render(dt);
	bg_image->render(dt);
	
}
