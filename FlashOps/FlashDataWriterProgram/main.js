const Serialport = require('serialport');
const fs = require('fs');

const port = new Serialport('COM8', {
    baudRate: 115200
});

fs.readFile('./L1-Flash1-Processed.csv', 'utf-8', async (err, data) => {
    if (err) throw err;
    let lines = data.split('\n');
    for(var i = 0; i < lines.length; i++) {
        lines[i] = lines[i].replace('\r', '');
    }
    for(var i=0; i < lines.length; i++) {
        port.write(lines[i] + '\n');
        console.log(lines[i] + '\n');
    }
});