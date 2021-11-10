BAGFILE=$1

rostopic echo -b  $BAGFILE -p /turbot/navigator/navigation->${BAGFILE%topi*}/turbot_navigation.csv
rostopic echo -b  $BAGFILE -p /turbot/modem_delayed->${BAGFILE%topic*}/turbot_modem_delayed.csv
rostopic echo -b  $BAGFILE -p /xiroi/usbllong->${BAGFILE%topic*}/turbot_usbllong.csv


