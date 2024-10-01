import os

city = 'Buffalo'
uncertainty = 'ALL'

folders = [x for x in os.listdir('scenario/TDCDP') if '.py' not in x \
                                                    and '.scn' not in x\
                                                    and '.DS' not in x
                                                    and '.DS_Store' not in x]

batchscn = open(f"scenario/TDCDP/{city}_{uncertainty}.scn", "w")

for folder in folders:
    files = [x for x in os.listdir(f'scenario/TDCDP/{folder}') if '.py' not in x \
                                                    and '.DS' not in x
                                                    and '.DS_Store' not in x]
    for file in files:    
        if city not in file:
            continue
        batchscn.write(f'00:00:00.00>SCEN {folder}_{file}\n')
        batchscn.write(f'00:00:00.00>PCALL TDCDP/{folder}/{file}\n')
        batchscn.write('00:00:00.00>FF\n')
        batchscn.write('00:00:00.00>SCHEDULE 12:00:00 HOLD\n\n')
