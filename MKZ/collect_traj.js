var map;

function initMap() {
    var latitude = 37.915567; // YOUR LATITUDE VALUE
    var longitude = -122.334844; // YOUR LONGITUDE VALUE

    var myLatLng = {lat: latitude, lng: longitude};

    map = new google.maps.Map(document.getElementById('map'), {
      center: myLatLng,
      zoom: 17,
      disableDoubleClickZoom: true, // disable the default map zoom on double click
    });

    //Add listener
    google.maps.event.addListener(map, "dblclick", function (event) {
        var latitude = event.latLng.lat();
        var longitude = event.latLng.lng();
        var direction = prompt("direction command", "2.0");
        console.log( latitude + ', ' + longitude + ', ' + direction);

    }); //end addListener


    // Update lat/long value of div when you move the mouse over the map
    google.maps.event.addListener(map,'mousemove',function(event) {
        document.getElementById('latmoved').innerHTML = event.latLng.lat();
        document.getElementById('longmoved').innerHTML = event.latLng.lng();
    });

    var marker = new google.maps.Marker({
      position: myLatLng,
      map: map,
      //title: 'Hello World'

      // setting latitude & longitude as title of the marker
      // title is shown when you hover over the marker
      title: latitude + ', ' + longitude
    });

    // Update lat/long value of div when the marker is clicked
    marker.addListener('click', function(event) {
      document.getElementById('latclicked').innerHTML = event.latLng.lat();
      document.getElementById('longclicked').innerHTML =  event.latLng.lng();
    });

    function readTextFile(file)
    {
        var rawFile = new XMLHttpRequest();
        rawFile.open("GET", file, false);
        rawFile.onreadystatechange = function ()
        {
            if(rawFile.readyState === 4)
            {
                if(rawFile.status === 200 || rawFile.status == 0)
                {
                    var allText = rawFile.responseText;
                    alert(allText);
                }
            }
        }
        rawFile.send(null);
    }


    google.maps.event.addListener(map, "rightclick", function(event) {
        var lat = event.latLng.lat();
        var lng = event.latLng.lng();
        // populate yor box/field with lat, lng
        alert(readTextFile("file:///Volumes/Data/Berkeley/code_and_data/code/aws/MKZ/traj1.txt"))
        alert("Lat=" + lat + "; Lng=" + lng);

        var marker = new google.maps.Marker({
          position: {lat: lat, lng: lng},
          map: map,
          // title is shown when you hover over the marker
          title: lat + ', ' + lng,
          label: 'A'
        });

    });

}


function drawMarker(lat, lng, command)
{
    var int2label = {2: "F", 3: "L", 4: "R", 5: "S"};
    var marker = new google.maps.Marker({
      position: {lat: lat, lng: lng},
      map: map,
      // title is shown when you hover over the marker
      title: lat + ', ' + lng,
      label: int2label[command]
    });
}

function loadFile(o)
{
    var fr = new FileReader();
    fr.onload = function(e)
        {
            showDataFile(e, o);
        };
    fr.readAsText(o.files[0]);
}

function showDataFile(e, o)
{
    //document.getElementById("data").innerText = e.target.result;
    var array = e.target.result.toString().split("\n")
    for(i in array) {
        var insider = array[i].split(", ");
        drawMarker(parseFloat(insider[0]), parseFloat(insider[1]), parseInt(insider[2]));
    }
}
