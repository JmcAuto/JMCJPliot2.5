import datetime
import os
import shutil
import sys
import collections




if __name__ == "__main__":
    protocol={}
    protocols=collections.OrderedDict()
    # protocols={}

    NameList=[]
    if len(sys.argv) != 2:
        #print "usage:\npython %s some_config.yml" % sys.argv[0]
        sys.exit(1)
    with open(sys.argv[1], 'r') as fp:
        NowName=[]
        firstSignal=True
        firstMessage=True
        firstSignalnum=0
        lastfirstSignalnum=0
        for line in fp.readlines():
            line = line.strip('\n')
            print "line%s" % line
            
            if "{" in line:
                x=line.split("{")
                
                firstSignalnum+=1
                if firstSignalnum==1:
                    NowName=[]
                NowName.append(x[0].strip(' ')+"_")
                firstSignal=True
                lastfirstSignalnum=firstSignalnum
                # print "NowName%s" % x[0].strip(' ')
            elif ":" in line:
                x=line.split(":")
                
                if firstSignal:
                    NowName.append(x[0].strip(' '))
                    firstSignal=False
                else:
                    # if len(NowName)!=0&lastfirstSignalnum==firstSignalnum:
                    #     NowName[-1]=x[0].strip(' ')
                    #     print "NowName%s" % x[0].strip(' ')
                    # elif lastfirstSignalnum!=firstSignalnum:
                    #     NowName.append(x[0].strip(' '))
                    if len(NowName)!=0:
                        NowName[-1]=x[0].strip(' ')
                    else:
                        NowName.append(x[0].strip(' '))
                # print "NowName%s" % x[0].strip(' ')
                title=""
                for y in NowName:
                    title=title+y
                    # print "y%s" % y
                if firstMessage:
                    NameList.append(title)
                    # print "NameList%s" % title
                else:
                    if title in NameList:
                        pass
                    else:
                        NameList.append(title)
                        


                print "title%s" % title
                # print "Data%s" % x[1].strip(' ')
                protocol[title]=x[1].strip(' ').strip('\n').strip('\r')
                # print "line%s" % line
            elif "}" in line:
                firstSignalnum-=1
                if len(NowName)!=0:
                    NowName.pop(-1)
                if len(NowName)!=0:
                    NowName.pop(-1)
            elif "---" in line:
                firstMessage=False
                protocols[protocol["header_timestamp_sec"]]=protocol
                print "header_timestamp_sec%s" % protocols[protocol["header_timestamp_sec"]]["header_timestamp_sec"]
                protocol={}
                NowName=[]
    with open("good"+sys.argv[1], 'w') as fp_write:
        tempLine=""
        for name in NameList:
             tempLine+=name+","

        tempLine+="\n"
        print "protocols%d" % len(protocols)
        for proto in protocols.values():
            print "proto%s" % proto["header_timestamp_sec"]
            for name in NameList:
                if proto.has_key(name):
                    tempLine+=proto[name]+","
                    print "name%s" % name
                    # print "data%s" % proto[name]
                else:
                    tempLine+=""+","
            tempLine+="\n"
            print "tempLine%s" % tempLine
            fp_write.write(tempLine)
            tempLine=""
