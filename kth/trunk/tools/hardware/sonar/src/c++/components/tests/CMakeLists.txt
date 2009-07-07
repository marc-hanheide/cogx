include_directories(../autogen)

add_cast_component(SonarPullClient SonarPullClient.cpp SonarPullClient.hpp)
link_cast_component(SonarPullClient Sonar)
