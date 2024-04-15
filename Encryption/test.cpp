#include "fileManagement.hpp"

using namespace std;




int main(){

    string command;
    string menu  = "<Encrypt File /e> <Decrypt File /d>";
    string path;
    vector<uint64_t> data;
    string str;
    
    
    while (true) {
        cout << menu << endl;
        cout<< ">>";
        getline(cin, command);

        if (command == "exit") {
            break;
        } 
        
        else if (command == "e") {
            cout<< "Enter Path To Input: ";
            getline(cin,path);
            data = readFileAsBinary(path);
            if(data == vector<uint64_t>{static_cast<uint64_t>(-1)}){
                break;
            }
            cout<< "Enter Encryption Password: ";
            string password;
            getline(cin,password);

            vector<uint8_t> key = stringToKey(password);
            
            Blowfish b(key);

            encryptData(b, data);
        
            cout << "Enter Path To Output: ";
            getline(cin,path);

            writeFileAsBinary(path, data);

            cout << "Encrypted!" <<endl;
            
        } 

        else if (command == "d") {
            cout<< "Enter Path To Input: ";
            getline(cin,path);

            data = readFileAsBinary(path);

            cout<< "Enter Encryption Password: ";
            string password;
            getline(cin,password);

            vector<uint8_t> key = stringToKey(password);

            Blowfish b(key);

            decryptData(b, data);

            cout << "Enter Path To Output: ";

            getline(cin,path);

            writeFileAsBinary(path,data);

            cout << "Decrypted!" <<endl;
        } 

        else {
            cout << "Unknown command.\n";
        }
    }

    cout << "Goodbye!\n";
}