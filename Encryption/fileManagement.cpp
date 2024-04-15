#include "fileManagement.hpp"


vector<uint64_t> readFileAsBinary(const string & path){

    vector<uint64_t> data;

    ifstream file(path, ios::binary | ios::ate);

    if(!file){
        cout<< "couldn't open file" << endl;
        data = {static_cast<uint64_t>(-1)};
        return data;
    }

    //Pointer is at end so tellg() returns size of file
    streamsize size = file.tellg();
    //Set pointer to beginning of data
    file.seekg(0, ios::beg);

    

    uint64_t block;

    //Add blocks of 64bit data 
    while(file.read(reinterpret_cast<char*>(&block), sizeof(block))){
        data.push_back(block);
    }
    
    //check for remainder i.e data that is not 64 bit that needs padding
    streamsize bytesLastRead = file.gcount();
    uint64_t paddingValue = sizeof(block) - bytesLastRead;

    //pad remainder of last 64 bit block with a value equal to how many values are being added
    if(bytesLastRead > 0 ){

        block <<= paddingValue * 8;

        for(streamsize i = 0 ; i < paddingValue ; i++){
            block |= paddingValue << (i * 8);
        }

        data.push_back(block);
    }

    //if no remainder then pad anyways with an extra block
    else if(bytesLastRead == 0){
        paddingValue = sizeof(block);
        block = 0;

        for(streamsize i = 0 ; i < paddingValue ; i++){
            block |= paddingValue << (i * 8);
        }

        data.push_back(block);

    }

    return data;


};




void writeFileAsBinary(const string & path, const vector<uint64_t> & data){
    //Wipe file if it already exists
    ofstream file(path, ios::binary | ios::trunc);

    for(int i = 0 ; i < data.size() - 1 ; i++){
        file.write(reinterpret_cast<const char*>(&data[i]), sizeof(uint64_t));
    }

    //handle last block seperately

    uint64_t lastBlock = data.back();
    //Extract size of padding used
    uint8_t paddingSize = lastBlock & 0xFF;

    int dataSizeLastBlock = sizeof(uint64_t) - paddingSize;

    file.write(reinterpret_cast<const char*>(&lastBlock), dataSizeLastBlock);

};

