# LED-Race
Weiterentwicklung eines LED-Races auf Arduino-basis
LED-RACE

In unserer Ausbildung haben wir ein LED-Race, bei welchem wir die Grundidee auch von GitHub haben, weiter entwickelt und mit ein paar zusätzlichen Funktionen ausgestattet.
Mit einem Arduino und einem Adressierbaren LED-Stripe haben wir ein LED-Rennen Gebaut, bei welchem man durch schnelles aufeinanderfolgendes tippen auf einen Taster seinen „wurm“ aus 5 LED´s nach vorne bewegen kann. Je nachdem wie schnell man auf den Taster tippt, desto schneller wird der „Wurm“ beschleunigt. Neben dieser Grundfunktion des LED-Rennen haben wir noch ein Boost-System entwickelt. Nach einer gewissen Zeit Lädt sich bei unserem optimierten rennen nämlich ein Boost auf, welcher durch einen Tastendruck auf dem 2. Taster am Kontroller aktiviert wird. Dadurch hat man dann für ein paar Sekunden die maximal erreichbare Geschwindigkeit. Der Status über den Boost, also wann dieser einsatzbereit ist, kann man anhand von LED´s sehen, bei welchen je nach boost Status immer eine LED mehr angeht. Ist der  Boost voll aufgeladen, dann blinkt diese anzeige dazu noch in Weiß.
An der Strecke, also an dem eigentlichen LED-Stripe, haben wir auch noch eine Verbesserung vorgenommen, indem wir die Strecke veränderbar gemacht haben. Der eigentliche Rundkurs hat also 2 Abzweigungen die einen weiteren Teil zu der Strecke hinzu fügen. Dieser Teil wird zu einem zufälligen Zeitpunkt in dem Rennen befahrbar. (Das Rennen geht mehrere Runden).
Dieser neue Teil macht es für den Spieler nicht anders das Spiel zu Spielen, macht aber Optisch einiges her.
Zuzüglich haben wir noch ein LC-Display verbaut, auf welchem man Vorab den Start-Countdown sehen kann und am Ende den Gewinner und die jeweiligen Punktzahlen der Spieler. Die Punktzahlen werden danach ermittelt, wie schnell jemand getippt hat. Durch diese Punktzahl wird das rennen deutlich vergleichbarer gemacht.

In den Dateien ist der  kommentierte Sourcecode für das Rennen zu finden Sowie auch ein Stromlaufplan für die zu erstellende Platine.
