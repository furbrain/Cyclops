// setup intersector jobbies
const observer = new IntersectionObserver((entries) => {
  entries.forEach(entry => {
    if (entry.isIntersecting) {
      console.debug('Img is now visible: ' + entry.target.dataset.rosTopic);
      entry.target.src = "/vid/stream?topic=" + entry.target.dataset.rosTopic + "&qos_profile=sensor_data&quality=50"
      // Run your function here
    } else {
      console.debug('Img is no longer visible: ' + entry.target.dataset.rosTopic);
      entry.target.src = "#"
    }
  });
});

// make video feeds dynamic
const imgs = document.querySelectorAll('.ros-img');
imgs.forEach(function(element) {
    observer.observe(element);
});