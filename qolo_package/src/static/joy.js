var WEBSOCKET_ROUTE = "/ws";

if(window.location.protocol == "http:"){
    //localhost
    var ws = new WebSocket("ws://" + window.location.host + WEBSOCKET_ROUTE);
    }
else if(window.location.protocol == "https:"){
    //Dataplicity
    var ws = new WebSocket("wss://" + window.location.host + WEBSOCKET_ROUTE);
    }


var JoyStick = (function(container, parameters) {
    ws.onopen = function(evt) {
    $("#ws-status").html("Connected");
    };

	ws.onmessage = function(evt) {
	    };

	ws.onclose = function(evt) {
	    $("#ws-status").html("Disconnected");
	    };


	parameters = parameters || {};
	var title = (undefined === parameters.title ? 'joystick' : parameters.title),
		width = (undefined === parameters.width ? 0 : parameters.width),
		height = (undefined === parameters.height ? 0 : parameters.height),
		internalFillColor = (undefined === parameters.internalFillColor ? '#00AA00' : parameters.internalFillColor),
		internalLineWidth = (undefined === parameters.internalLineWidth ? 2 : parameters.internalLineWidth),
		internalStrokeColor = (undefined === parameters.internalStrokeColor ? '#003300' : parameters.internalStrokeColor),
		externalLineWidth = (undefined === parameters.externalLineWidth ? 2 : parameters.externalLineWidth),
		externalStrokeColor = (undefined === parameters.externalStrokeColor ? '#008000' : parameters.externalStrokeColor);
	
	// Create Canvas element and add it in the Container object
	var objContainer = document.getElementById(container);
	var canvas = document.createElement('canvas');
	canvas.id = title;
	if(width == 0) width = objContainer.clientWidth;
	if(height == 0) height = objContainer.clientHeight;
	canvas.width = width;
	canvas.height = height;
	objContainer.appendChild(canvas);
	var context=canvas.getContext('2d');
	
	var pressed = 0; // Bool - 1=Yes - 0=No
	var circumference = 2 * Math.PI;
	var internalRadius = (canvas.width-((50*2)+10))/2;
	var maxMoveStick = internalRadius + 5;
	var externalRadius = internalRadius + 30;
	var centerX = canvas.width / 2;
	var centerY = canvas.height / 2;
	var directionHorizontalLimitPos = canvas.width / 10;
	var directionHorizontalLimitNeg = directionHorizontalLimitPos * -1;
	var directionVerticalLimitPos = canvas.height / 10;
	var directionVerticalLimitNeg = directionVerticalLimitPos * -1;
	// Used to save current position of stick
	var movedX=centerX;
	var movedY=centerY;
		
	// Check if the device support the touch or not
	if("ontouchstart" in document.documentElement)
	{
		canvas.addEventListener('touchstart', onTouchStart, false);
		canvas.addEventListener('touchmove', onTouchMove, false);
		canvas.addEventListener('touchend', onTouchEnd, false);
	}
	else
	{
		canvas.addEventListener('mousedown', onMouseDown, false);
		canvas.addEventListener('mousemove', onMouseMove, false);
		canvas.addEventListener('mouseup', onMouseUp, false);
	}
	// Draw the object
	drawExternal();
	drawInternal(centerX, centerY);
	/******************************************************
	 * Private methods
	 *****************************************************/
	/**
	 * @desc Draw the external circle used as reference position
	 */
	function drawExternal()
	{
		context.beginPath();
		context.arc(centerX, centerY, externalRadius, 0, circumference, false);
		context.lineWidth = externalLineWidth;
		context.strokeStyle = externalStrokeColor;
		context.stroke();
	}
	/**
	 * @desc Draw the internal stick in the current position the user have moved it
	 */
	function drawInternal()
	{
		context.beginPath();
		if(movedX<internalRadius) movedX=maxMoveStick;
		if((movedX+internalRadius)>canvas.width) movedX=canvas.width-(maxMoveStick);
		if(movedY<internalRadius) movedY=maxMoveStick;
		if((movedY+internalRadius)>canvas.height) movedY=canvas.height-(maxMoveStick);
		context.arc(movedX, movedY, internalRadius, 0, circumference, false);
		// create radial gradient
		var grd = context.createRadialGradient(centerX, centerY, 5, centerX, centerY, 200);
		// Light color
		grd.addColorStop(0, internalFillColor);
		// Dark color
		grd.addColorStop(1, internalStrokeColor);
		context.fillStyle = grd;
		context.fill();
		context.lineWidth = internalLineWidth;
		context.strokeStyle = internalStrokeColor;
		context.stroke();
	}
	
	/**
	 * @desc Events for manage touch
	 */
	function onTouchStart(event) 
	{
		pressed=1;
	}
	function onTouchMove(event)
	{
		// Prevent the browser from doing its default thing (scroll, zoom)
		event.preventDefault();
		if(pressed==1)
		{
			movedX=event.touches[0].pageX;
			movedY=event.touches[0].pageY;
			// Manage offset
			movedX-=canvas.offsetLeft;
			movedY-=canvas.offsetTop;
			// Delete canvas
			context.clearRect(0, 0, canvas.width, canvas.height);
			// Redraw object
			drawExternal();
			drawInternal();
		}
	} 
	function onTouchEnd(event) 
	{
		pressed=0;
		// Reset position store variable
		movedX=centerX;
		movedY=centerY;
		// Delete canvas
		context.clearRect(0, 0, canvas.width, canvas.height);
		// Redraw object
		drawExternal();
		drawInternal();
		//canvas.unbind('touchmove');
	}
	/**
	 * @desc Events for manage mouse
	 */
	function onMouseDown(event) 
	{
		pressed=1;
	}
	function onMouseMove(event) 
	{
		if(pressed==1)
		{
			movedX=event.pageX;
			movedY=event.pageY;
			// Manage offset
			movedX-=canvas.offsetLeft;
			movedY-=canvas.offsetTop;
			// Delete canvas
			context.clearRect(0, 0, canvas.width, canvas.height);
			// Redraw object
			drawExternal();
			drawInternal();
		}
	}
	function onMouseUp(event) 
	{
		pressed=0;
		// Reset position store variable
		movedX=centerX;
		movedY=centerY;
		// Delete canvas
		context.clearRect(0, 0, canvas.width, canvas.height);
		// Redraw object
		drawExternal();
		drawInternal();
		//canvas.unbind('mousemove');
	}
	/******************************************************
	 * Public methods
	 *****************************************************/
	/**
	 * @desc The width of canvas
	 * @return Number of pixel width 
	 */
	this.GetWidth = function () 
	{
		return canvas.width;
	};
	
	/**
	 * @desc The height of canvas
	 * @return Number of pixel height
	 */
	this.GetHeight = function () 
	{
		return canvas.height;
	};
	
	/**
	 * @desc The X position of the cursor relative to the canvas that contains it and to its dimensions
	 * @return Number that indicate relative position
	 */
	this.GetPosX = function ()
	{
		return movedX;
	};
	
	/**
	 * @desc The Y position of the cursor relative to the canvas that contains it and to its dimensions
	 * @return Number that indicate relative position
	 */
	this.GetPosY = function ()
	{
		return movedY;
	};
	
	/**
	 * @desc Normalizzed value of X move of stick
	 * @return Integer from -100 to +100
	 */
	this.GetX = function ()
	{
		ws.send("GetX = " + (-100*((movedX - centerX)/maxMoveStick)).toFixed() + "GetY = " + ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed());
		return (-100*((movedX - centerX)/maxMoveStick)).toFixed();
	};

	/**
	 * @desc Normalizzed value of Y move of stick
	 * @return Integer from -100 to +100
	 */
	this.GetY = function ()
	{
		return ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed();
	};
	
	/**
	 * @desc Get the direction of the cursor as a string that indicates the cardinal points where this is oriented
	 * @return String of cardinal point N, NE, E, SE, S, SW, W, NW and C when it is placed in the center
	 */
	this.GetDir = function()
	{
		var result = "";
		var orizontal = movedX - centerX;
		var vertical = movedY - centerY;
		
		if(vertical>=directionVerticalLimitNeg && vertical<=directionVerticalLimitPos)
		{
			result = "C";
		}
		if(vertical<directionVerticalLimitNeg)
		{
			result = "N";
		}
		if(vertical>directionVerticalLimitPos)
		{
			result = "S";
		}
		
		if(orizontal<directionHorizontalLimitNeg)
		{
			if(result=="C")
			{ 
				result = "W";
			}
			else
			{
				result += "W";
			}
		}
		if(orizontal>directionHorizontalLimitPos)
		{
			if(result=="C")
			{ 
				result = "E";
			}
			else
			{
				result += "E";
			}
		}
		
		return result;
	};
});
