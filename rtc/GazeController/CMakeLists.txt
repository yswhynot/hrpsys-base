set(comp_sources GazeController.cpp GazeControllerService_impl.cpp ../Stabilizer/TwoDofController.cpp ../Stabilizer/Integrator.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(GazeController SHARED ${comp_sources})
target_link_libraries(GazeController ${libs})
set_target_properties(GazeController PROPERTIES PREFIX "")

add_executable(GazeControllerComp GazeControllerComp.cpp ${comp_sources})
target_link_libraries(GazeControllerComp ${libs})

set(target GazeController GazeControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
