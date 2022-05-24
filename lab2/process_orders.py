#!/usr/bin/env python3
import sys;

if __name__ == "__main__":

    ulaz = open(sys.argv[1],"r");
    izlaz = open("orders_report.txt","w");

    orders = {};

    number_of_items = 0;
    
    while(1==1):

        if not ulaz.readline():

            break;
        else:

            number_of_items += 1;
            number_of_articles = int(ulaz.readline());

            for i in range(number_of_articles):

                article = ulaz.readline();
                item = article.split(":")[0];
                quantity = float(article.split(":")[1][1:])

                if(item in orders.keys()):

                    orders[item] = orders[item] + quantity;

                else:

                    orders[item] = quantity;

    izlaz.write("Processing orders.txt\n");
    izlaz.write("Number of orders: {}\n".format(number_of_items));
    izlaz.write("Item totals: \n");

    print("Processing orders.txt")
    print("Number of orders: {}".format(number_of_items));
    print("Item totals: ");

    for key in orders:

        izlaz.write("   {}: {}\n".format(key,orders[key]));
        print("   {}: {}".format(key,orders[key]));

    ulaz.close();
    izlaz.close();
