# script file
# git for RoboRTS/decision from user:kidding, run and input username password
# folder your folder example /home/dji/chenwang
# run in terminal: bash xxx.sh
rm -rf /home/dji/chenwang/decision
cp -r /home/dji/roborts_ws/src/decision /home/dji/chenwang/
cd /home/dji/chenwang/
git fetch origin master
git merge origin/master
git add ./decision
git commit -m "decision update"
git push -u origin master
