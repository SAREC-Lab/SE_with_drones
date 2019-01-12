#cd "$(dirname "$(realpath "$0")")"

#change to match your repository path...
REPO=/home/uav/git/Dronology-Community/


gnome-terminal --working-directory=$REPO/$server -x bash -c "mvn clean; mvn install;bash"

