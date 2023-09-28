# script file
# git for RoboRTS/decision from user:kidding, run and input username password
# folder your folder example /home/dji/chenwang
# run in terminal: bash xxx.sh
cd /home/dji/chenwang
git pull origin master
git add ./decision/
cp -r ./decision /home/dji/roborts_ws/src/
