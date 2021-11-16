# postprocessing
Postprocessing robot data

## To obtain the csvs desired files use 
usbldata_csv_extractor.py script execute by doing: 

python usbldata_csv_extractor.py path_to_the_bagfile

Once the csvs are extracted save them following this structure:

Desired path + turbot + 
                             modem_delayed":"modem_delayed.csv
                             modem_raw":"modem_raw.csv
                             USBLlon":"USBLlon.csv
                             nav_status_turbot":"nav_status_tu.csv
                
Desired path + xiroi + nav_status_xi.csv           

## To process csvs information use:
process_usbl_data_good.ipynb 
