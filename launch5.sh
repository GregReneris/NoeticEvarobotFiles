echo "Launch 5 terminals"

gnome-terminal -- bash term1.sh


echo "Press enter to launch next window"
read x
gnome-terminal -- bash term2.sh

echo "Press enter to launch next window"
read x
gnome-terminal -- bash term3.sh

echo "Press enter to launch next window"
read x
gnome-terminal -- bash term4.sh

echo "Press enter to launch next window"
read x
gnome-terminal -- bash term5.sh


echo "Done."
