import csv
from datetime import datetime


def _csv_write(data_fetcher, filename):
    """
    This function is a private function to write data to a CSV until
    termination.
    :param data_fetcher: A DataFetcher object which describes which data to write to the CSV.
    :param filename: The filename to write the CSV to.
    :return: None
    """
    file = open(filename, 'w')
    
    writer = csv.writer(file)
    writer.writerow(data_fetcher.get_header())

    while True:
        line = list()
        line.append(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        
        for x in data_fetcher:
            line.append(x)
        writer.writerow(line)


def create_csv(data_fetcher, filename):
    """
    This function creates a CSV and then starts writing to the CSV until termination.
    :param data_fetcher: A DataFetcher object which describes which data to write to the CSV.
    :param filename: The filename to write the CSV to.
    :return: None
    """
    if filename is None:
        filename = "data_" + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + '.csv'

    _csv_write(data_fetcher, filename)


def csv_write(data_fetcher, filename):
    """
    This is the publicly called function to write data to a CSV.
    :param data_fetcher: A DataFetcher object which describes which data to write to the CSV.
    :param filename: The filename to write the CSV to.
    :return: None
    """
    create_csv(data_fetcher, filename)
