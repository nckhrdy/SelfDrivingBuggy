//Nicholas Hardy, Riya Deokar, Marybel Boujaoude, Hassan Hijazi
const http = require("http");
const dgram = require("dgram");

const PORT = 8081;
const HOST = "192.168.1.36";

const server = dgram.createSocket("udp4");

const udp = dgram.createSocket("udp4");

const message1 = "start";

http
  .createServer(function (req, res) {
    res.writeHead(200, { "Content-Type": "text/html" });
res.write("<html>");
res.write("<head>");
res.write("<title>Buggy Remote</title>");
res.write("<style>");
res.write("body { background-color: #f2f2f2; }");
res.write("h1 { color: blue; text-align: center; margin-top: 50px; }");
res.write("#btn {");
res.write("display: block;");
res.write("width: 300px;");
res.write("height: 80px;");
res.write("margin: 0 auto;");
res.write("background-color: #4CAF50;");
res.write("color: white;");
res.write("font-size: 24px;");
res.write("border-radius: 10px;");
res.write("}");
res.write("</style>");
res.write("</head>");
res.write("<body>");
res.write("<h1>Buggy Remote</h1>");
res.write("<button id='btn' type='button' onclick='changeText()'>Remote Start!</button>");
res.write("<script>");
res.write("function changeText() {");
res.write("var btn = document.getElementById('btn');");
res.write("if (btn.innerHTML === 'Buggy On') {");
res.write("btn.innerHTML = 'Buggy Off';");
res.write("} else {");
res.write("btn.innerHTML = 'Buggy On';");
res.write("}");
res.write("}");
res.write("</script>");
res.write("</body>");
res.write("</html>");
  })
  .listen(PORT);
server.on("listening", function () {
  const address = server.address();
  console.log(
    "UDP Server listening on " + address.address + ":" + address.port
  );
});
server.on("message", function (message, remote) {
  console.log("" + message);
  messageFromEsp += message.toString() + "\n";
});
udp.send(message1, 8081, "192.168.1.37", (err) => {
  if (err) {
    console.error(err);
  } else {
    console.log("UDP message sent to ESP");
  }
});
server.bind(PORT, HOST);
