#!/usr/bin/env python3
import sys;

if __name__ == "__main__":

    ulazna_datoteka = sys.argv[1];
    izlazna_datoteka = sys.argv[2];

    ulaz = open(ulazna_datoteka,"r");
    izlaz = open(izlazna_datoteka,"w");

    lines = ulaz.readlines();
    lista = [];

    x = float(input("Enter floating point number: "));

    lista = [float(line)+x for line in lines];

    for l in lista:
        izlaz.writelines(str(l)+"\n");

    print(lista);

    ulaz.close();
    izlaz.close();

