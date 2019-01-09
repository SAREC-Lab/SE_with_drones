#cd "$(dirname "$(realpath "$0")")"

#change to match your repository path...
REPO=/win/Work/git/Dronology-ICSE-GA/
#source $(Compile-Dronology.sh export)


#don't change things below (unless you know what you are doing)...
server=edu.nd.dronology.services.launch
vaadin=edu.nd.dronology.ui.vaadin
#pkill -9 java
#pkill -9 python 
gnome-terminal --working-directory=$REPO/$server -x bash -c "mvn exec:java"
gnome-terminal --working-directory=$REPO/$vaadin -x bash -c "mvn jetty:run"


## starts the GCS UI remove this line if you want to start the GCS via the json config file
#gnome-terminal --working-directory=$GCS -x bash -c "python main_ui.py"
#uncomment this 2 lines if you want to start the GCS with the json file...
#cd $GCS
#gnome-terminal --working-directory=$GCS -x bash -c "python main.py -gid mygid -addr localhost -p 1234 -d ../cfg/drone_cfgs/$DRONECONFIG"

sleep 15
xdg-open 'http://localhost:8080/vaadinui'





