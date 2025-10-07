//
// Dummy-Datei als Platzhalter, kann gelöscht werden, wenn neue Dateien im Ordner src erstellt wurden
//

#include "dummy.h"

Dummy::Dummy(int zahl) : zahl(zahl) {

}

Dummy::~Dummy() {

}

int Dummy::add(int zahl) {
        return this->zahl + zahl;
}