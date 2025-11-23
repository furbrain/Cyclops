var frames = ['/imu', '/map'];

// Create a map (dictionary) of TFClients
var tfClients = {};

// Initialize each TFClient with a different fixed frame
frames.forEach(function(frame) {
  tfClients[frame] = new ROSLIB.ROS2TFClient({
    ros: ros,
    angularThres: 0.01,
    transThres: 0.01,
    rate: 10.0,
    fixedFrame: frame
  });
});

function addPointClouds(viewer) {
    const topics = viewer.dataset.pointclouds.split(/\s+/).filter(t => t);
    frame = viewer.dataset.frame || "/imu";
    viewer.pointClouds = [];
    size = (viewer.dataset.pointsize || 1.0) + 0.0;
    topics.forEach(function(topic) {
        viewer.pointClouds.push(new ROS3D.PointCloud2({
          ros: ros,
          tfClient: tfClients[frame],
          rootObject: viewer.viewer.scene,
          topic: topic,
          material: { size: size, color: 0xff00ff }
        }));
    });
}

function addMarkers(viewer) {
    const topics = viewer.dataset.markers.split(/\s+/).filter(t => t);
    frame = viewer.dataset.frame || "/imu";
    viewer.markers = [];
    topics.forEach(function(topic) {
        viewer.markers.push(new ROS3D.MarkerClient({
          ros: ros,
          tfClient: tfClients[frame],
          rootObject: viewer.viewer.scene,
          topic: topic
        }));
    });
}

const viewer_observer = new IntersectionObserver((entries) => {
  entries.forEach(entry => {
    if (entry.isIntersecting) {
      console.debug('Viewer is now visible: ' + entry.target.id);
      entry.target.viewer.start();
    } else {
      console.debug('Viewer is no longer visible: ' + entry.target.id);
      entry.target.viewer.stop();
    }
  });
});


const viewers = document.querySelectorAll('div.ros-viewer');
viewers.forEach(function(viewer_div) {
    console.log("Processing viewer: "+ viewer_div.id)
    scale = viewer_div.dataset.scale || 10;
    viewer_div.viewer = new ROS3D.Viewer({
      divID : viewer_div.id,
      width : 800,
      height : 600,
      antialias : true,
      cameraZoomSpeed : 1.0,
      cameraPose: {x: scale, y: scale, z: scale}
    });
    viewer_div.grid = new ROS3D.Grid();
    viewer_div.viewer.addObject(viewer_div.grid);
    addPointClouds(viewer_div);
    addMarkers(viewer_div);
    viewer_observer.observe(viewer_div);
});
