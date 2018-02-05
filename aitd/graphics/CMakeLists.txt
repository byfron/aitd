add_library(GraphicsCore
  Shader.hpp
  Shader.cpp
  Mesh.hpp
  Mesh.cpp
  Camera.hpp
  Camera.cpp
  GraphicsEngine.hpp
  GraphicsEngine.cpp
  RenderSystem.hpp
  RenderSystem.cpp)


target_link_libraries(GraphicsCore
  poly2tri
  clip2tri
  ${BGFX_LIBRARIES})

  
target_include_directories(GraphicsCore PUBLIC
  ${SDL_INCLUDE_DIR})

add_subdirectory(clip2tri)
