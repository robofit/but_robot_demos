Voice control
=============

Ma dve casti - serverovou (zachycuje data z kinectu) a klientskou (rozpoznavani), komunikuji pres socket.

Pozadavky
---------
*   [libfreenect](https://github.com/OpenKinect/libfreenect/)


Server
------
Obcas blbne pri nacitani firmware (hodi segfault), staci ho jednou spustit pres valgrind a je to.
Jinak se nachazi ve slozce `src/cpp`, tam se pomoci `make` preklada a spousti.

Klient
------
Klient se preklada antem v korenovem adresari, spousti se pomoci `java -cp lib/sphinx4.jar:bin/bc.jar fit_vut.BC.BC`.
Je to kvuli tomu, ze ant nevypisuje veci hned, obcas si pocka az ke konci programu (coz je celkem neprakticke).
Da se nakonfigurovat gramatika (data/gram/digits.gram), je to soubor formatu [JSGF](http://en.wikipedia.org/wiki/JSGF).

Poznamky
--------
*   Zatim je to napsane jen tak nahrubo, ale funguje to
