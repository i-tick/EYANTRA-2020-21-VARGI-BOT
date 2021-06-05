#!/usr/bin/env python

print("hello")
import datetime
import json
import requests
url="https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fsblj4me2-frBx9rzoMh8tJAiOgmU/2/public/full?alt=json"
ssContent1=requests.get(url).json()
d=[]

print("Loop 1")
while 'entry' not in ssContent1['feed'].keys():
    url="https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fsblj4me2-frBx9rzoMh8tJAiOgmU/2/public/full?alt=json"
    ssContent1=requests.get(url).json()
    # print(ssContent1)

i=0
print("Loop 2")
while i<9:
    print("Loop 3")
    while len(ssContent1['feed']['entry']) <i+1:
        url="https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fsblj4me2-frBx9rzoMh8tJAiOgmU/2/public/full?alt=json"
        ssContent1=requests.get(url).json()
    # print(ssContent1['feed']['entry'][i]['content'])
    data_values = ssContent1['feed']['entry'][i]['content'].values()
    val = data_values[0].encode("utf-8")
    str_values = list(val.split(","))
    data_separated=[]
    for str_value in str_values:
        each_val = str_value.split(":",1)
        each_val[1] = each_val[1][1:]
        data_separated.append(each_val)
    print(data_separated)
    date_dt3 = datetime.datetime.strptime(data_separated[3][1], '%Y-%m-%d %H:%M:%S')
    # print(date_dt3)

    i+=1    