#pragma once

#include "World.hpp"
#include "ScriptingManager.hpp"
#include "entities/Entity.hpp"
#include "entities/System.hpp"
#include <map>

namespace aitd {

class AITDEngine {
public:
	
	AITDEngine();
	~AITDEngine();
	void init();
	void createSubsystems();
	void loadGameData();
	void requestQuit();
	
	template <typename S>
	void add(std::shared_ptr<S> system) {
		system_manager->add<S>(system);
	}
	
	void run_frame(float dt) {
		system_manager->update_all(dt);
	}

	static Entity::Id player_entity_id;

	//TODO: hopefully we can get rid of those after migrating the logic
	static std::map<int, int16> globals;
	static std::map<int, int16> C_globals;
	static std::map<uint32_t, int> CameraEntityToIndex;
	
protected:

	bool running = false;
	
	EntityManager::Ptr entity_manager;
	EventManager::Ptr event_manager;
	SystemManager::Ptr system_manager;
	ScriptingManager::Ptr scripting_manager;
	
	World::Ptr world;
	
};
}
