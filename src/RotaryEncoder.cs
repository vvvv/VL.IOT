using System.Device.Gpio;

namespace Iot.Device.RotaryEncoder
{
    /// <summary>
    /// Binding that exposes a working quadrature rotary encoder
    /// </summary>
    public class WorkingRotaryEncoder : IDisposable
    {
        private GpioController _controller;
        private int _pinA;
        private int _pinB;
        private bool _disposeController = true;

        /// <summary>
        /// The number of pulses expected per rotation of the encoder
        /// </summary>
        public int PulsesPerRotation { get; private set; }

        /// <summary>
        /// The number of pulses before or after the start position of the encoder
        /// </summary>
        public long PulseCount { get; set; }

        /// <summary>
        /// The number of rotations backwards or forwards from the initial position of the encoder
        /// </summary>
        public float Rotations { get => (float)PulseCount / PulsesPerRotation; }

        /// <summary>
        /// EventHandler to allow the notification of value changes.
        /// </summary>
        public event EventHandler<RotaryEncoderEventArgs>? PulseCountChanged;

        /// <summary>
        /// WorkingRotaryEncoder constructor
        /// </summary>
        /// <param name="pinA">Pin A that is connected to the rotary encoder. Sometimes called clk</param>
        /// <param name="pinB">Pin B that is connected to the rotary encoder. Sometimes called data</param>
        /// <param name="edges">The pin event types to 'listen' for.</param>
        /// <param name="pulsesPerRotation">The number of pulses to be received for every full rotation of the encoder.</param>
        /// <param name="controller">GpioController that hosts Pins A and B.</param>
        /// <param name="shouldDispose">True to dispose the controller</param>
        public WorkingRotaryEncoder(int pinA, int pinB, PinEventTypes edges, int pulsesPerRotation, GpioController? controller = null, bool shouldDispose = true)
        {
            _disposeController = controller == null | shouldDispose;
            _controller = controller ?? new GpioController();

            PulsesPerRotation = pulsesPerRotation;
            Initialize(pinA, pinB, edges);
        }

        /// <summary>
        /// WorkingRotaryEncoder constructor
        /// </summary>
        /// <param name="pinA">Pin A that is connected to the rotary encoder. Sometimes called clk</param>
        /// <param name="pinB">Pin B that is connected to the rotary encoder. Sometimes called data</param>
        /// <param name="pulsesPerRotation">The number of pulses to be received for every full rotation of the encoder.</param>
        public WorkingRotaryEncoder(int pinA, int pinB, int pulsesPerRotation)
            : this(pinA, pinB, PinEventTypes.Falling, pulsesPerRotation, new GpioController(), false)
        {
        }

        /// <summary>
        /// Modify the current value on receipt of a pulse from the rotary encoder.
        /// </summary>
        /// <param name="blnUp">When true then the value should be incremented otherwise it should be decremented.</param>
        /// <param name="milliSecondsSinceLastPulse">The number of miliseconds since the last pulse.</param>
        protected virtual void OnPulse(bool blnUp, int milliSecondsSinceLastPulse)
        {
            PulseCount += blnUp ? 1 : -1;

            // fire an event if an event handler has been attached
            PulseCountChanged?.Invoke(this, new RotaryEncoderEventArgs(PulseCount));
        }

        // Values returned by 'process'
        // No complete step yet.
        private const byte DIR_NONE = 0x0;
        // Clockwise step.
        private const byte DIR_CW = 0x10;
        // Anti-clockwise step.
        private const byte DIR_CCW = 0x20;

        private const byte R_START = 0x0;
        private byte state = R_START;

        //// Use the half-step state table (emits a code at 00 and 11)
        //private const byte R_CCW_BEGIN = 0x1;
        //private const byte R_CW_BEGIN = 0x2;
        //private const byte R_START_M = 0x3;
        //private const byte R_CW_BEGIN_M = 0x4;
        //private const byte R_CCW_BEGIN_M = 0x5;

        //private readonly byte[,] haltstep_table = new byte[,]
        //{
        //    // R_START (00)
        //    {R_START_M,           R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
        //    // R_CCW_BEGIN
        //    {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
        //    // R_CW_BEGIN
        //    {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
        //    // R_START_M (11)
        //    {R_START_M,           R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
        //    // R_CW_BEGIN_M
        //    {R_START_M,           R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
        //    // R_CCW_BEGIN_M
        //    {R_START_M,           R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
        //};

        private const byte R_CW_FINAL = 0x1;
        private const byte R_CW_BEGIN = 0x2;
        private const byte R_CW_NEXT = 0x3;
        private const byte R_CCW_BEGIN = 0x4;
        private const byte R_CCW_FINAL = 0x5;
        private const byte R_CCW_NEXT = 0x6;

        private readonly byte[,] fullstep_table = new byte[,]
        {
            { R_START, R_CW_BEGIN, R_CCW_BEGIN, R_START },
            { R_CW_NEXT, R_START, R_CW_FINAL, R_START | DIR_CW },
            { R_CW_NEXT, R_CW_BEGIN, R_START, R_START },
            { R_CW_NEXT, R_CW_BEGIN, R_CW_FINAL, R_START },
            { R_CCW_NEXT, R_START, R_CCW_BEGIN, R_START },
            { R_CCW_NEXT, R_CCW_FINAL, R_START, R_START | DIR_CCW },
            { R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START }
        };

        /// <summary>
        /// Initialize a WorkingRotaryEncoder
        /// </summary>
        /// <param name="pinA">Pin A that is connected to the rotary encoder. Sometimes called clk</param>
        /// <param name="pinB">Pin B that is connected to the rotary encoder. Sometimes called data</param>
        /// <param name="edges">The pin event types to 'listen' for.</param>
        private void Initialize(int pinA, int pinB, PinEventTypes edges)
        {
            _pinA = pinA;
            _pinB = pinB;

            _controller.OpenPin(_pinA, PinMode.Input);
            _controller.OpenPin(_pinB, PinMode.Input);

            _controller.RegisterCallbackForPinValueChangedEvent(_pinA, edges, (o, e) =>
            {
                // Grab the state of input pins.
                byte pinstate = (byte)(((byte)_controller.Read(_pinB) << 1) | (byte)_controller.Read(_pinA));
                //Console.WriteLine($"state: {state} - pin state: {pinstate}");

                // Determine the new state from the pins and state table.
                state = fullstep_table[state & 0xf, pinstate];

                // Return emit bits, i.e., the generated event.
                var dir = state & 0x30;

                if (dir == DIR_CW)
                    OnPulse(true, 0);
                else if (dir == DIR_CCW)
                    OnPulse(false, 0);
            });

            _controller.RegisterCallbackForPinValueChangedEvent(_pinB, edges, (o, e) =>
            {
                // Grab the state of input pins.
                byte pinstate = (byte)(((byte)_controller.Read(_pinB) << 1) | (byte)_controller.Read(_pinA));
                //Console.WriteLine($"state: {state} - pin state: {pinstate}");

                // Determine the new state from the pins and state table.
                state = fullstep_table[state & 0xf, pinstate];

                // Return emit bits, i.e., the generated event.
                var dir = state & 0x30;

                if (dir == DIR_CW)
                    OnPulse(true, 0);
                else if (dir == DIR_CCW)
                    OnPulse(false, 0);
            });
        }

        /// <inheritdoc/>
        public void Dispose()
        {
            _controller?.ClosePin(_pinA);
            _controller?.ClosePin(_pinB);

            if (_disposeController)
            {
                _controller?.Dispose();
            }
        }
    }
}
