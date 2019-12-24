var tl=gsap.timeline(); 
var controller=new ScrollMagic.Controller();
gsap.to(".about-container",{
    opacity:0
});
var tween=gsap.from(".about-container", {
    opacity:0, y:200, duration: 1, ease: "power1" 
});
  var scene=new ScrollMagic.Scene({
      triggerElement:".about-container",
      triggerHook: 1
   })
  .setTween(tween)
  .addIndicators()
  .addTo(controller);