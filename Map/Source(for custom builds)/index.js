import "ol/ol.css";
import * as olProj from "ol/proj";
import Feature from "ol/Feature";
import Map from "ol/Map";
import { unByKey } from "ol/Observable";
import View from "ol/View";
import { easeOut } from "ol/easing";
import Point from "ol/geom/Point";
import { Tile as TileLayer, Vector as VectorLayer } from "ol/layer";
import { fromLonLat } from "ol/proj";
import { OSM, Vector as VectorSource } from "ol/source";
import { Circle as CircleStyle, Stroke, Style } from "ol/style";
import { getVectorContext } from "ol/render";
import "roslib"
var tileLayer = new TileLayer({
  source: new OSM({
    wrapX: false
  })
});

var map = new Map({
  layers: [tileLayer],
  target: "map",
  view: new View({
    center: olProj.fromLonLat([87.309032, 22.315826]),
    zoom: 17.88,
    multiWorld: true
  })
});

var source = new VectorSource({
  wrapX: false
});
var vector = new VectorLayer({
  source: source
});
map.addLayer(vector);

function addRandomFeature(x,y) {
    console.log('New point Found!!');
  document.getElementById("output").innerHTML =
    document.getElementById("output").innerHTML +
    "<br>" +
    "Point found at x= " +
    x +
    " y= " +
    y;
  var geom = new Point(fromLonLat([x, y]));
  var feature = new Feature(geom);
  source.addFeature(feature);
}

var duration = 3000;
function flash(feature) {
  var start = new Date().getTime();
  var listenerKey = tileLayer.on("postrender", animate);

  function animate(event) {
    var vectorContext = getVectorContext(event);
    var frameState = event.frameState;
    var flashGeom = feature.getGeometry().clone();
    var elapsed = frameState.time - start;
    var elapsedRatio = elapsed / duration;
    // radius will be 5 at start and 30 at end.
    var radius = easeOut(elapsedRatio) * 25 + 5;
    var opacity = easeOut(1 - elapsedRatio);

    var style = new Style({
      image: new CircleStyle({
        radius: radius,
        stroke: new Stroke({
          color: "rgba(255, 0, 0, " + opacity + ")",
          width: 0.25 + opacity
        })
      })
    });

    vectorContext.setStyle(style);
    vectorContext.drawGeometry(flashGeom);
    if (elapsed > duration) {
      unByKey(listenerKey);
      return;
    }
    // tell OpenLayers to continue postrender animation
    map.render();
  }
}

source.on("addfeature", function(e) {
  flash(e.feature);
});

    //window.setInterval(addRandomFeature, 2000);
  //  addRandomFeature(87.309032, 22.315826);

var ros = new ROSLIB.Ros();

  // If there is an error on the backend, an 'error' emit will be emitted.
  ros.on('error', function(error) {
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'none';
    document.getElementById('error').style.display = 'inline';
    console.log(error);
  });

  // Find out exactly when we made a connection.
  ros.on('connection', function() {
    console.log('Connection made!');
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('error').style.display = 'none';
    document.getElementById('closed').style.display = 'none';
    document.getElementById('connected').style.display = 'inline';
  });

  ros.on('close', function() {
    console.log('Connection closed.');
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'inline';
  });

  // Create a connection to the rosbridge WebSocket server.
  ros.connect('ws://localhost:9090');
     var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/listener',
    messageType : 'std_msgs/Float64MultiArray'
  });

  // Then we add a callback to be called every time a message is published on this topic.
var xy = [];
  listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);

if ((xy.find(function(value){ 
        if(value[0]==message.data[0]&&value[1]==message.data[1])
            return 1;

}))==undefined)
 {
   addRandomFeature(message.data[0],message.data[1]);
    xy.push([message.data[0],message.data[1]]);
}
  });


