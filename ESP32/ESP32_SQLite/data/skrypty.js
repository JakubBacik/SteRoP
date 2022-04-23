$( document ).ready(function() {
  var temperature = [];
  var pressure = [];
  var humidity = [];
  var sekundy = [];
  
  var ctx = document.getElementById('wykres');

  var wykres = new Chart(ctx, {
    type: 'line',
    data:
    {
      labels: sekundy,
      datasets:
      [
      { 
          data: temperature,
          yAxisID: 'A',
          label: 'Temperature',
          borderColor: 'rgba(360, 84, 50, 1)',
          backgroundColor: 'rgba(255, 130, 150, 0.2)'
      },{
          data: pressure,
          yAxisID: 'B',
          label:'Pressure',
          borderColor: 'rgba(36 , 238, 89, 1)',
          backgroundColor: 'rgba(36 , 238, 89, 0.2)',
      },{
        data: humidity,
        yAxisID: 'C',
        label:'Humidity',
        borderColor: 'rgba(23, 126, 214, 1)',
        backgroundColor: 'rgba(23, 126, 214, 0.2)',
    }
      ]
    },
    options: {
      tooltips: {
        mode: 'nearest'
      },
      scales: {
        yAxes: [{
          id: 'A',
          type: 'linear',
          position: 'left',
          ticks: {
            min: 0,
            max: 50,
            stepSize: 10,
            fontColor: 'rgba(360, 84, 50, 1)',
            callback: function(value, index, values) {
              return value + '°C';
            }
          }
        }, {
          id: 'B',
          type: 'linear',
          position: 'right',
          ticks: {
            min: 900,
            max: 1300,
            stepSize: 100,
            fontColor: 'rgba(36 , 238, 89, 1)',
            callback: function(value, index, values) {
              return value + ' hPa';
            }
          }
        }, {
          id: 'C',
          type: 'linear',
          position: 'right',
          ticks: {
            min: 0,
            max: 100,
            stepSize: 10,
            fontColor: 'rgba(23, 126, 214, 1)',
            callback: function(value, index, values) {
              return value + ' %';
            }
          }
        }]
      },
      elements: {
        line: {
          tension: 0, // disables bezier curves
        },
        point: {
          radius: 2,
          borderWidth: 2,
          pointStyle: 'circle'
        }
      }
  
    }
  });

  function addData(chart, data, dset) {
    chart.data.datasets[dset].data.push(data);
    chart.update();
  }
  
  function addLabel(chart, label) {
    chart.data.labels.push(label);
    chart.update();
  }
  
  function popData(chart, index) {
    chart.data.labels.splice(index, 1);
    chart.data.datasets.forEach((dataset) => {
      dataset.data.splice(index, 1);
    });
    chart.update();
  }
  function showPlot(zawartosc){
    var dane = zawartosc.split("\n");
    for (var i = dane.length-2; i >-1; i--) {
      var cos = dane[i].split("!");
      addLabel(wykres, cos[5]);
      addData(wykres, cos[3], 0);
      addData(wykres, cos[1], 1);
      addData(wykres, cos[2], 2);
    }
  }

  
  var req = new XMLHttpRequest();
  req.open('GET', "/dates", false); 
  
  req.onreadystatechange = function () {
  if (req.readyState == 4) {
        var zawartosc = this.responseText;
        showPlot(zawartosc);
    }
  };
  setInterval(function(){
    var req2 = new XMLHttpRequest();
    req2.open('GET', "/currentdates", false);
    req2.onreadystatechange = function () {
    if (req2.readyState == 4) {
      var zawartosc = this.responseText;
      var cos = zawartosc.split("!");
      addLabel(wykres, cos[5]);
      addData(wykres, cos[3], 0);
      addData(wykres, cos[1], 1);
      addData(wykres, cos[2], 2);
      popData(wykres, 0);
    }
  }
  req2.send(null);   
  }, 60000);
  req.send(null);              
}); 


function showDataBase()
{
  var req = new XMLHttpRequest();
  req.open('GET', "/dates", false);
  req.onreadystatechange = function () {
    if (req.readyState == 4) {
      var zawartosc = this.responseText;
      var dane = zawartosc.split("\n");
      var table = document.getElementById("myTable");
      var j=1;
      for (var i = dane.length-2; i >-1; i--) {
        var row = table.insertRow(j);
        var column = dane[i].split("!");
        row.insertCell(0).innerHTML = column[0];
        row.insertCell(1).innerHTML = column[4];
        row.insertCell(2).innerHTML = column[5];
        row.insertCell(3).innerHTML = column[3];
        row.insertCell(4).innerHTML = column[1];
        row.insertCell(5).innerHTML = column[2]; 
        j++;   
      }
    }
  }
  req.send(null); 
};


function showData(){
  var req2 = new XMLHttpRequest();
  req2.open('GET', "/currentdates", false);
  req2.onreadystatechange = function () {
    if (req2.readyState == 4) {
      var zawartosc = this.responseText;
      var dane = zawartosc.split("!");
      document.getElementById('temperature').innerHTML = dane[3];
      document.getElementById('humidity').innerHTML = dane[2];
      document.getElementById('presure').innerHTML = dane[1];
    }
  }
  req2.send(null);
  
};

function getCurrentData(){
  var req2 = new XMLHttpRequest();
  req2.open('GET', "/currentdates", false);
  req2.onreadystatechange = function () {
    if (req2.readyState == 4) {;
      var zawartosc1 = this.responseText; 
      var table1 = document.getElementById("myTable");
      var rowCount = table1.rows.length;
      var row1 = table1.deleteRow(1);
      var row1 = table1.insertRow(rowCount-1);
      var column1 = zawartosc1.split("!");
        row1.insertCell(0).innerHTML = column1[0];
        row1.insertCell(1).innerHTML = column1[4];
        row1.insertCell(2).innerHTML = column1[5];
        row1.insertCell(3).innerHTML = column1[3];
        row1.insertCell(4).innerHTML = column1[1];
        row1.insertCell(5).innerHTML = column1[2];
   }
 }
 req2.send(null);
};