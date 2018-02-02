#pragma once
#include <ActorLoader.hpp>
#include <entities/Entity.hpp>

namespace aitd {
class AITDViewer {
public:

	~AITDViewer();
	void init();
	void run_frame(float dt);	
	
protected:
	
	EntityManager::Ptr entity_manager;
};
}
