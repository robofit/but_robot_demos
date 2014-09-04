=== Analyza kvality cesty ===

Predmet: ROB

Autor: Ondrej Novotny, xnovot96@stud.fit.vutbr.cz

Zprovozneni:
Do kodu je nutne v souboru SpeedControl.h upravit prikaz pro nastaveni maximalni rychlosti pomoci dynamickych parametru. Cesta se v nastaveni ve skolni verzi ROS lisi oproti predpokladane jenz je dostupna v dokumentaci.
Je nutne pred spustenim nastavit parametry pomoci parametrickeho serveru v ros, viz soubor rob_project.yaml. Nastavit do cesty /rob_project.

Uzel je vytvoren pomoci catkin.


Vyznam parametru:
frame_length_road - delka ona pro vypocet kvality cesty
frame_length_angle - delka okna pro vypocet uhlu
g - hodnota mereni zrychleni (1, 10)
gth - prak pro pocitani pretizeni (pretizeni pod prahem se ignoruje)
max_speed_road - maximalni rychlost pro analyzu cesty
a_speed_road - koeficient exponencionalni funkce pro kvalitu cesty
a_speed_ax - koeficient exponencionalni funkce pro naklon v ose x
a_speed_ay - koeficient exponencionalni funkce pro naklon v ose y
max_acc -maximallni mozne zrychleni
a_acc - koeficient exponencionalni funkce pro zrychleni
auto - priznak ze je robot autonomne rizen

