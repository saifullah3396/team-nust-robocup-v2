# CMake generated Testfile for 
# Source directory: /home/muptii/Documents/robocup/team-nust-robocup-v2/resources/jsoncpp/src/jsontestrunner
# Build directory: /home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/build-local/team-nust-robocup/resources/jsoncpp/src/jsontestrunner
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(jsoncpp_readerwriter "/usr/bin/python2" "-B" "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/jsoncpp/src/jsontestrunner/../../test/runjsontests.py" "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/cross/bin/jsontestrunner_exe" "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/jsoncpp/src/jsontestrunner/../../test/data")
set_tests_properties(jsoncpp_readerwriter PROPERTIES  WORKING_DIRECTORY "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/jsoncpp/src/jsontestrunner/../../test/data")
add_test(jsoncpp_readerwriter_json_checker "/usr/bin/python2" "-B" "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/jsoncpp/src/jsontestrunner/../../test/runjsontests.py" "--with-json-checker" "/home/muptii/Documents/robocup/team-nust-robocup-v2/build-V5/Release-Motion/cross/bin/jsontestrunner_exe" "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/jsoncpp/src/jsontestrunner/../../test/data")
set_tests_properties(jsoncpp_readerwriter_json_checker PROPERTIES  WORKING_DIRECTORY "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/jsoncpp/src/jsontestrunner/../../test/data")