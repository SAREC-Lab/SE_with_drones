#cd "$(dirname "$(realpath "$0")")"

#change to match your repository path...
REPO=/win/Work/git/Dronology-ICSE-GA/


gnome-terminal --working-directory=$REPO/$server -x bash -c "mvn clean; mvn install;bash"

