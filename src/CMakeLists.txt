set(SRC_FILES
  common.h
  gfx.h gfx.cpp
  actor.h actor.cpp
  textes.h textes.cpp
  main.h main.cpp
  actor_list.cpp actor_list.h aitd_box.cpp aitd_box.h anim_action.cpp anim_action.h 
  anim.cpp anim.h common.h config.h cos_table.cpp cos_table.h debug_font.cpp debug_font.h 
  debugger.cpp debugger.h eval_var.cpp eval_var.h file_access.cpp file_access.h 
  filestream.cpp fitd.cpp fitd.h floor.cpp floor.h fmopl.cpp fmopl.h font.cpp font.h hqr.cpp hqr.h input.cpp input.h 
  inventory.cpp inventory.h life.cpp life_macro_table.cpp life.h life_2.cpp life_2.h lines.cpp 
  main_loop.cpp main_loop.h music.cpp music.h object.cpp object.h 
  osystem.h osystemSDL.cpp osystemSDL_GL.cpp palette.h palette.cpp pak.cpp pak.h polys.cpp 
  resource.cpp resource.h renderer.cpp renderer.h room.cpp room.h sequence.cpp save.cpp save.h screen.cpp screen.h 
  startup_menu.cpp startup_menu.h system_menu.cpp system_menu.h tatou.cpp tatou.h 
  common/textconsole.h thread_code.cpp thread_code.h 
  time.cpp track.cpp track.h unpack.cpp unpack.h vars.cpp vars.h 
  version.cpp version.h video_mode.cpp video_mode.h zv.cpp zv.h common/textconsole.cpp 
)

add_library(fitd ${SRC_FILES})
target_link_libraries(fitd
  z
  ${SDL_LIBRARY}
  SDL_mixer
  GL
  GLU
  )

add_executable(fitd_aitd aitd.cpp)
target_link_libraries(fitd_aitd
  fitd)

target_include_directories(fitd PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${SDL_INCLUDE_DIR})

