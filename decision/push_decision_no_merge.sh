# script file
# git for RoboRTS/decision from user:kidding, run and input username password
# folder your folder example /home/dji/chenwang
# run in terminal: bash xxx.sh
sudo rm -rf /home/dji/chenwang/decision
sudo cp -r /home/dji/roborts_ws/src/decision /home/dji/chenwang/
cd /home/dji/chenwang/
git add ./decision
git commit -m "decision update"
git push -u origin master
