BAGFILE=$1

rostopic echo -b $BAGFILE -p /xiroi/navigator/navigation->${BAGFILE%topi*}/xiroi_navigation.csv
rostopic echo -b $BAGFILE -p /turbot/modem_delayed->${BAGFILE%topi*}/xiroi_modem_delayed.csv
rostopic echo -b $BAGFILE -p /xiroi/usbllong->${BAGFILE%topi*}/xiroi_usbllong.csv