// This code is based on source code from an article by Stephen Cleary on
// MSDN magazine: http://msdn.microsoft.com/en-us/magazine/dn630647.aspx
//
// Licensed under the Microsoft Limited Public License. The full text of
// this license can be found at http://clrinterop.codeplex.com/license
// and is also included in this distribution.

using System;
using System.ComponentModel;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;
using System.Windows;
using System.Windows.Interactivity;

namespace KinetisUpdater.ViewModel
{
    // A base class for ViewModel classes
    public class ObservableObject : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        protected void RaisePropertyChangedEvent(string propertyName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }
    
    }

    public sealed class DelegateCommand : ICommand
    {
        private readonly Action _command;

        public DelegateCommand(Action command)
        {
            _command = command;
        }

        public void Execute(object parameter)
        {
            _command();
        }

        bool ICommand.CanExecute(object parameter)
        {
            return true;
        }

        event EventHandler ICommand.CanExecuteChanged
        {
            add { }
            remove { }
        }
    }

    public sealed class DelegateCommand<T> : ICommand
    {
        private readonly Action<T> _command;

        public DelegateCommand(Action<T> command)
        {
            _command = command;
        }

        public void Execute(object parameter)
        {
            _command((T)parameter);
        }

        bool ICommand.CanExecute(object parameter)
        {
            return true;
        }

        event EventHandler ICommand.CanExecuteChanged
        {
            add { }
            remove { }
        }
    }

    public interface IAsyncCommand : ICommand
    {
        Task ExecuteAsync(object parameter);
    }

    public abstract class AsyncCommandBase : ObservableObject, IAsyncCommand
    {
        public abstract bool CanExecute(object parameter);

        public abstract Task ExecuteAsync(object parameter);

        public async void Execute(object parameter)
        {
            await ExecuteAsync(parameter);
        }

        public event EventHandler CanExecuteChanged
        {
            add { CommandManager.RequerySuggested += value; }
            remove { CommandManager.RequerySuggested -= value; }
        }

        protected void RaiseCanExecuteChanged()
        {
            CommandManager.InvalidateRequerySuggested();
        }
    }

    public class AsyncCommand<TResult> : AsyncCommandBase
    {
        private readonly Func<CancellationToken, Task<TResult>> _command;
        private readonly CancelAsyncCommand _cancelCommand;
        private NotifyTaskCompletion<TResult> _execution;

        public AsyncCommand(Func<CancellationToken, Task<TResult>> command)
        {
            _command = command;
            _cancelCommand = new CancelAsyncCommand();
        }

        public override bool CanExecute(object parameter)
        {
            return Execution == null || Execution.IsCompleted;
        }

        public override async Task ExecuteAsync(object parameter)
        {
            _cancelCommand.NotifyCommandStarting();
            Execution = new NotifyTaskCompletion<TResult>(_command(_cancelCommand.Token));
            RaiseCanExecuteChanged();
            await Execution.TaskCompletion;
            _cancelCommand.NotifyCommandFinished();
            RaiseCanExecuteChanged();
        }

        public ICommand CancelCommand
        {
            get { return _cancelCommand; }
        }

        public NotifyTaskCompletion<TResult> Execution
        {
            get { return _execution; }
            private set
            {
                _execution = value;
                RaisePropertyChangedEvent("Execution");
            }
        }

        private sealed class CancelAsyncCommand : ICommand
        {
            private CancellationTokenSource _cts = new CancellationTokenSource();
            private bool _commandExecuting;

            public CancellationToken Token { get { return _cts.Token; } }

            public void NotifyCommandStarting()
            {
                _commandExecuting = true;
                if (!_cts.IsCancellationRequested)
                    return;
                _cts = new CancellationTokenSource();
                RaiseCanExecuteChanged();
            }

            public void NotifyCommandFinished()
            {
                _commandExecuting = false;
                RaiseCanExecuteChanged();
            }

            bool ICommand.CanExecute(object parameter)
            {
                return _commandExecuting && !_cts.IsCancellationRequested;
            }

            void ICommand.Execute(object parameter)
            {
                _cts.Cancel();
                RaiseCanExecuteChanged();
            }

            public event EventHandler CanExecuteChanged
            {
                add { CommandManager.RequerySuggested += value; }
                remove { CommandManager.RequerySuggested -= value; }
            }

            private void RaiseCanExecuteChanged()
            {
                CommandManager.InvalidateRequerySuggested();
            }
        }
    }

    public static class AsyncCommand
    {
        public static AsyncCommand<object> Create(Func<Task> command)
        {
            return new AsyncCommand<object>(async _ => { await command(); return null; });
        }

        public static AsyncCommand<TResult> Create<TResult>(Func<Task<TResult>> command)
        {
            return new AsyncCommand<TResult>(_ => command());
        }

        public static AsyncCommand<object> Create(Func<CancellationToken, Task> command)
        {
            return new AsyncCommand<object>(async token => { await command(token); return null; });
        }

        public static AsyncCommand<TResult> Create<TResult>(Func<CancellationToken, Task<TResult>> command)
        {
            return new AsyncCommand<TResult>(command);
        }
    }

    public sealed class InvokeCommandActionWithEventArgs : TriggerAction<DependencyObject>
    {

        public static readonly DependencyProperty CommandProperty = DependencyProperty.Register("Command", typeof(ICommand), typeof(InvokeCommandActionWithEventArgs), null);
        public static readonly DependencyProperty CommandParameterProperty = DependencyProperty.Register("CommandParameter", typeof(object), typeof(InvokeCommandActionWithEventArgs), null);
        public static readonly DependencyProperty ParameterProperty = DependencyProperty.Register("Parameter", typeof(object), typeof(InvokeCommandActionWithEventArgs), null);

        public ICommand Command
        {
            get
            {
                return (ICommand)base.GetValue(InvokeCommandActionWithEventArgs.CommandProperty);
            }
            set
            {
                base.SetValue(InvokeCommandActionWithEventArgs.CommandProperty, value);
            }
        }

        public object CommandParameter
        {
            get
            {
                return base.GetValue(InvokeCommandActionWithEventArgs.CommandParameterProperty);
            }
            set
            {
                base.SetValue(InvokeCommandActionWithEventArgs.CommandParameterProperty, value);
            }
        }
        
        public object Parameter
        {
            get
            {
                return this.GetValue(ParameterProperty);
            }
            set
            {
                this.SetValue(ParameterProperty, value);
            }
        }

        protected override void Invoke(object parameter)
        {
            this.Parameter = parameter;
            if ((this.AssociatedObject != null) &&(Command != null) && Command.CanExecute(this.CommandParameter))
            {
                Command.Execute(this.CommandParameter);
            }
        }
    }//class InvokeCommandActionWithEventArgs

    // A simple class to filter the key inputs.
    public class KeyInputFilter
    {
        //return false: not ignore this key input. 
        //       true:  ignore this key input.
        public static Boolean Filter(Key KeyInput, ModifierKeys KeyModifier)
        {
            // Key 0 to 9
            if (KeyInput >= Key.D0 && KeyInput <= Key.D9 && (KeyModifier & ModifierKeys.Shift) != ModifierKeys.Shift)
                return false;
            // Key 0 to 9 at number pad
            else if (KeyInput >= Key.NumPad0 && KeyInput <= Key.NumPad9)
                return false;
            // Key a(A) to f(F) and x(X)
            else if (KeyInput == Key.A || KeyInput == Key.B || KeyInput == Key.C || KeyInput == Key.D || KeyInput == Key.E || KeyInput == Key.F || KeyInput == Key.X)
                return false;
            // Key Delete, BackSpace, Home, End, Insert and Esc
            else if (KeyInput == Key.Delete || KeyInput == Key.Back || KeyInput == Key.Home || KeyInput == Key.End || KeyInput == Key.Insert || KeyInput == Key.Escape)
                return false;
            // Key PgUp, PgDn, <- and ->
            else if (KeyInput == Key.PageDown || KeyInput == Key.PageUp || KeyInput == Key.Left || KeyInput == Key.Right)
                return false;
            // ignore other key inputs
            else
                return true;
        }
        //if the input is x(X), ignore it.
        public static Boolean FilterX(Key KeyInput)
        {
            if (KeyInput == Key.X)
                return true;
            else
                return false;
        }
    }//class KeyInputFilter
}//namespace KinetisUpdater.ViewModel
