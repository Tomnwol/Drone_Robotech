Help to use logs :
    tac log.csv | sed -n '1,/^\*\*\*/p' | tac
    ->return datas from last flight

    place these datas in data.csv file and launch python program

    Alternatively, you can use this command to store directly:
    tac log.csv | sed -n '1,/^\*\*\*/p' | tac | sed '1d' > data.csv

