
var ledON = 0;
var fs = require('fs');
fs.writeFile('data.csv', 'Temperature,Voltage,AccelX,AccelY,AccelZ,Roll,Pitch\n', function(err){
 if(err) throw err;
 console.log('File created')
})


// Required modules
var dgram = require('dgram');

// Port and IP
var PORT = 3333;
var HOST = '10.0.0.115';

// Create socket
var server = dgram.createSocket('udp4');

// Create UDP server
server.on('listening', function () {
   var address = server.address();
   console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
   console.log(remote.address + ':' + remote.port +' - ' + message);
   //console.log(message);
   // Send Ok acknowledgement
   if(ledON == 0){
     server.send("Ok!",remote.port,remote.address,function(error){
       if(error){
         console.log('MEH!');
       }
       else{
         console.log('Sent: Ok!');
       }
     });
   }
   else{
     server.send("Toggle!",remote.port,remote.address,function(error){
       if(error){
         console.log('MEH!');
       }
       else{
         console.log('Sent: Ok!');
       }
     });
     ledON = 0;
   }



   // Append data to CSV file
   fs.appendFile('data.csv', message + "\n", function(err){
     if(err) throw err;
     console.log('Data appended')
   })

});

// Bind server to port and IP
server.bind(PORT, HOST);


/////////////////////////////////////////////////////////////////////////////////////
//  Read from .csv file, pipe data to HTML  /////////////////////////////////////////

/* This code modified from https://expressjs.com/en/starter/hello-world.html */
const csv = require('csv-parser');
const express = require('express');
const app = express();
var path = require('path');
const HTMLport = 3000;

/* Sends HTML file from server to browser */
app.get('/', (req, res) => {
 res.sendFile(path.join(__dirname + '/hurricane_box.html'));
})

/* Specifies $.get("/data") in HTML file */
app.get('/data', (req, res) => {

 var data = [];  // Array to hold CSV data

 fs.createReadStream('data.csv')   // create readStream from fs(?)
 .pipe(csv())            // Pipe data to csv object (inside server object)

 .on('data', (row) => {
   data.push(row);   // Appends each row to data array
 })

 .on('end', () => {
   res.send(data);   // Send CSV data to HTML page when done
 });
})

server.on('message', function (message, remote) {
 app.get("/my-url", function(req, res) {
     // Log data
     console.log(req.query);

     // Respond to the client
     res.send("LED Toggled");

     // Toggle ON bit
     ledON ^= 1;
 });
});


app.listen(HTMLport, () => {
 console.log(`App listening at http://localhost:${HTMLport}`)
})
