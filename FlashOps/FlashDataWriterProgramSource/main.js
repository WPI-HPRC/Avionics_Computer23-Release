const Serialport = require('serialport');
const fs = require('fs');
let configName = "./config.json"
let rawData = fs.readFileSync(configName);
let config = JSON.parse(rawData);

const port = new Serialport(config.portName, {
    baudRate: 115200
});

fs.readFile(config.fileName, 'utf-8', async (err, data) => {
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