#include "AITDEngine.hpp"
#include <graphics/GraphicsEngine.hpp>

class AITD : public aitd::GraphicsEngine {
public:
	AITD(const char* name, const char* desc) : aitd::GraphicsEngine(name, desc) {
		//auto console = spd::stdout_logger_mt("Client");
	}

	void init_engine() override {
		engine.init();
	}

	void frame(const float dt) override {
		engine.run_frame(dt);
	}

private:

	aitd::AITDEngine engine;
	
};

ENTRY_IMPLEMENT_MAIN(AITD, "AITD", "Alone In The Dark");

